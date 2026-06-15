# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Record a LeRobot dataset for dual CRP arms (GP teleop @ 80Hz, write @ dataset.fps).

Cameras: configure any subset of ``top``, ``left_wrist``, ``right_wrist`` on ``--robot.cameras``
(only include devices that enumerate successfully).
Observation: CRP joints + UI50 + UI56–58 (read-only feedback) + images.
Action: CRP joints + UI50 only.
"""

import logging
from dataclasses import asdict
from pprint import pformat

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.common.control_utils import (
    sanity_check_dataset_name,
    sanity_check_dataset_robot_compatibility,
)
from lerobot.configs import parser
from lerobot.datasets import (
    VideoEncodingManager,
    aggregate_pipeline_dataset_features,
    create_initial_features,
)
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import make_default_processors
from lerobot.robots import make_robot_from_config
from lerobot.robots.crp_arm_dual.crp_arm_dual import CRPArmDual
from lerobot.robots.crp_arm_dual.sdk import exit_process_after_dual_disconnect
from lerobot.scripts.crp_gp.config import DualGPTiming, RecordDualCRPConfig, teleop_cfg_from_record
from lerobot.scripts.crp_gp.loop import (
    TARGET_FRAME_NAME,
    align_dual_gp_for_episode,
    lock_gps_to_current_tcp,
    prepare_dual_gp_session,
)
from lerobot.scripts.crp_gp.mappers.so101_urdf import SO101_ARM_MOTOR_NAMES, resolve_so101_urdf_path
from lerobot.scripts.crp_gp.record_utils import (
    assert_local_dataset_for_resume,
    crp_dual_hw_action_features,
    crp_dual_hw_observation_features,
    prepare_new_dataset_root,
    resolve_orbbec_top_camera,
)
from lerobot.scripts.crp_gp.record import (
    ignore_signals_during_cleanup,
    new_episode_events,
    record_loop_crp_dual,
    stdin_episode_hotkeys,
    wait_reset_between_episodes,
)
from lerobot.scripts.crp_gp.ui_probe import GripperUiProbeManager, apply_ui_probe_connect_policy
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.feature_utils import combine_feature_dicts
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot.utils.utils import init_logging, log_say
from lerobot.utils.visualization_utils import init_rerun

logger = logging.getLogger(__name__)


def _build_dataset_features(robot: CRPArmDual, *, use_videos: bool) -> dict:
    teleop_action_processor, _, robot_observation_processor = make_default_processors()
    return combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=teleop_action_processor,
            initial_features=create_initial_features(action=crp_dual_hw_action_features()),
            use_videos=use_videos,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                observation=crp_dual_hw_observation_features(robot)
            ),
            use_videos=use_videos,
        ),
    )


@parser.wrap()
def record_dual_crp(cfg: RecordDualCRPConfig) -> LeRobotDataset:
    init_logging()
    logger.info(pformat(asdict(cfg)))

    if cfg.display_data:
        init_rerun(session_name="recording")

    resolve_orbbec_top_camera(cfg.robot)
    robot = make_robot_from_config(cfg.robot)
    if not isinstance(robot, CRPArmDual):
        raise TypeError(f"Expected CRPArmDual, got {type(robot)}")

    teleop = make_teleoperator_from_config(cfg.teleop)
    _, _, robot_observation_processor = make_default_processors()
    dataset_features = _build_dataset_features(robot, use_videos=cfg.dataset.video)
    num_cameras = len(robot.cameras)

    if cfg.resume:
        assert_local_dataset_for_resume(cfg)
        dataset = LeRobotDataset.resume(
            cfg.dataset.repo_id,
            root=cfg.dataset.root,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            camera_encoder=cfg.dataset.camera_encoder,
            encoder_threads=cfg.dataset.encoder_threads,
            streaming_encoding=cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
            image_writer_processes=cfg.dataset.num_image_writer_processes if num_cameras > 0 else 0,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras
            if num_cameras > 0
            else 0,
        )
        sanity_check_dataset_robot_compatibility(
            dataset, robot, cfg.dataset.fps, dataset_features
        )
    else:
        sanity_check_dataset_name(cfg.dataset.repo_id, policy_cfg=None)
        prepare_new_dataset_root(cfg)
        dataset = LeRobotDataset.create(
            cfg.dataset.repo_id,
            cfg.dataset.fps,
            root=cfg.dataset.root,
            robot_type=robot.name,
            features=dataset_features,
            use_videos=cfg.dataset.video,
            image_writer_processes=cfg.dataset.num_image_writer_processes,
            image_writer_threads=cfg.dataset.num_image_writer_threads_per_camera * num_cameras,
            batch_encoding_size=cfg.dataset.video_encoding_batch_size,
            camera_encoder=cfg.dataset.camera_encoder,
            encoder_threads=cfg.dataset.encoder_threads,
            streaming_encoding=cfg.dataset.streaming_encoding,
            encoder_queue_maxsize=cfg.dataset.encoder_queue_maxsize,
        )

    ui_probe: GripperUiProbeManager | None = None
    if cfg.gripper_ui_probe:
        ui_probe = GripperUiProbeManager(hz=cfg.gripper_ui_probe_hz)
    apply_ui_probe_connect_policy(robot, ui_probe)

    teleop.connect()
    try:
        robot.connect()
    except Exception:
        teleop.disconnect()
        try:
            robot.disconnect()
        except Exception:
            pass
        raise

    urdf_path = resolve_so101_urdf_path()
    kinematics = RobotKinematics(
        urdf_path=urdf_path,
        target_frame_name=TARGET_FRAME_NAME,
        joint_names=list(SO101_ARM_MOTOR_NAMES),
    )
    tele_cfg = teleop_cfg_from_record(cfg)
    events = new_episode_events()

    interrupted = False
    recorded_episodes = 0
    arms: tuple | None = None
    try:
        prepare_dual_gp_session(robot, tele_cfg, ui_probe=ui_probe)
        with stdin_episode_hotkeys(events), VideoEncodingManager(dataset):
            total = cfg.dataset.num_episodes
            while recorded_episodes < total and not events["stop_recording"]:
                ep_human = recorded_episodes + 1
                log_say(
                    f"Recording episode {ep_human} of {total} (dataset index {dataset.num_episodes})",
                    cfg.play_sounds,
                )
                arms, _ = align_dual_gp_for_episode(
                    teleop, robot, kinematics, tele_cfg, arms=arms
                )
                timing = DualGPTiming()

                result = record_loop_crp_dual(
                    robot=robot,
                    teleop=teleop,
                    kinematics=kinematics,
                    cfg=cfg,
                    tele_cfg=tele_cfg,
                    arms=arms,
                    timing=timing,
                    events=events,
                    dataset=dataset,
                    robot_observation_processor=robot_observation_processor,
                    control_time_s=cfg.dataset.episode_time_s,
                    single_task=cfg.dataset.single_task,
                    ui_probe=ui_probe,
                    display_data=cfg.display_data,
                )

                if events["rerecord_episode"]:
                    log_say("Re-record episode", cfg.play_sounds)
                    events["rerecord_episode"] = False
                    events["exit_early"] = False
                    dataset.clear_episode_buffer()
                    if not events["stop_recording"]:
                        log_say("Reset before re-record", cfg.play_sounds)
                        reset_result = wait_reset_between_episodes(
                            robot,
                            reset_time_s=cfg.dataset.reset_time_s,
                            events=events,
                            label="Re-record reset",
                            dataset=dataset,
                        )
                        if reset_result.aborted:
                            logger.error(
                                "CRP disconnected during re-record reset; stopping session."
                            )
                            break
                    continue

                if result.aborted:
                    logger.error(
                        "CRP disconnected during episode; discarding buffer and stopping session."
                    )
                    dataset.clear_episode_buffer()
                    break

                if result.frames == 0:
                    logger.warning("Episode ended with 0 recorded frames; skipping save.")
                    dataset.clear_episode_buffer()
                    continue

                log_say("Saving episode", cfg.play_sounds)
                logger.info(
                    "Saving episode (GP idle, CRP servo stays on; encoding may take a while)."
                )
                events["hotkeys_paused"] = True
                try:
                    # Sequential video encoding: parallel ffmpeg workers often OOM with 2+ cameras.
                    dataset.save_episode(parallel_encoding=False)
                finally:
                    events["hotkeys_paused"] = False
                recorded_episodes += 1
                logger.info(
                    "Episode saved (%d/%d). Press q in this terminal to stop all, "
                    "or continue with next episode.",
                    recorded_episodes,
                    total,
                )

                if events["stop_recording"]:
                    logger.info(
                        "stop_recording was set during save; ending session after episode %d.",
                        recorded_episodes,
                    )

                if not events["stop_recording"] and recorded_episodes < total:
                    log_say("Reset the environment", cfg.play_sounds)
                    reset_result = wait_reset_between_episodes(
                        robot,
                        reset_time_s=cfg.dataset.reset_time_s,
                        events=events,
                        dataset=dataset,
                    )
                    if reset_result.aborted:
                        logger.error(
                            "CRP disconnected during reset; stopping session "
                            "(episode %d already saved).",
                            recorded_episodes,
                        )
                        break

        log_say("Stop recording", cfg.play_sounds, blocking=True)
    except KeyboardInterrupt:
        interrupted = True
        logging.info("Recording interrupted (Ctrl+C); discarding in-progress episode buffer.")
        writer = getattr(dataset, "writer", None)
        if writer is not None:
            writer.stop_image_writer()
        dataset.clear_episode_buffer()
    finally:
        with ignore_signals_during_cleanup():
            if ui_probe is not None:
                ui_probe.stop_all()
            try:
                lock_gps_to_current_tcp(robot, teleop_cfg_from_record(cfg))
            except Exception as exc:
                logging.warning("GP lock on exit failed: %s", exc)
            robot.disconnect()
            teleop.disconnect()

    if interrupted:
        log_say("Recording stopped", cfg.play_sounds)
        exit_process_after_dual_disconnect(exit_code=130)

    if cfg.dataset.push_to_hub:
        if recorded_episodes <= 0:
            logger.warning("Skipping push_to_hub: no episodes were saved.")
        else:
            try:
                dataset.push_to_hub(tags=cfg.dataset.tags, private=cfg.dataset.private)
            except Exception as exc:
                logger.error(
                    "push_to_hub failed (%s). Dataset remains on disk at %s. "
                    "Check HF login and proxy settings (e.g. unset ALL_PROXY if socks fails).",
                    exc,
                    dataset.root,
                )

    log_say("Exiting", cfg.play_sounds)
    exit_process_after_dual_disconnect()
    return dataset


def main() -> None:
    register_third_party_plugins()
    record_dual_crp()


if __name__ == "__main__":
    main()
