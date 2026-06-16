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
Load ``third_party/CrpRobotPy`` for dual CRP (``CRPArmDual``).

Prepends the SDK dir to ``LD_LIBRARY_PATH``, optionally fixes ``DT_RUNPATH`` on
``CrpRobotPy.so``, and attaches ``CrpRobotPatch.so`` only for legacy vendor builds
that lack native dual-arm reads / UI.

Build from ``~/python_C++/CrpRobotPy`` (``set_GJs_second``, ``read_*_second``, etc.)
and deploy ``CrpRobotPy.so`` to ``third_party/CrpRobotPy/``.
"""

from __future__ import annotations

import ctypes
import logging
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

_crp_runpath_state: dict[str, str] = {}
_dual_patch_installed = False
_sdk_loaded = False
_vendor_dual_exit_hook_registered = False


def exit_process_after_dual_disconnect(exit_code: int = 0) -> None:
    """Exit without running CrpRobotPy destructors after explicit dual disconnect."""
    if os.environ.get("CRP_DUAL_NO_OS_EXIT", "").strip().lower() in ("1", "true", "yes"):
        return
    os._exit(exit_code)


def register_vendor_dual_process_exit() -> None:
    """Avoid segfault when CrpRobotPy destructor runs after dual disconnect at process exit."""
    global _vendor_dual_exit_hook_registered
    if _vendor_dual_exit_hook_registered:
        return
    if os.environ.get("CRP_DUAL_NO_OS_EXIT", "").strip().lower() in ("1", "true", "yes"):
        return
    import atexit

    atexit.register(os._exit, 0)
    _vendor_dual_exit_hook_registered = True


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "third_party" / "CrpRobotPy").is_dir():
            return parent
    raise RuntimeError("Cannot find third_party/CrpRobotPy from repo root.")


def _crp_sdk_dir(third_party_root: Path | None = None) -> Path:
    root = third_party_root if third_party_root is not None else _repo_root() / "third_party"
    return (root / "CrpRobotPy").resolve()


def _prepend_env_lib_path(dirs: list[str]) -> list[str]:
    env_key = "PATH" if sys.platform == "win32" else "LD_LIBRARY_PATH"
    sep = os.pathsep if sys.platform == "win32" else ":"

    head = [os.path.abspath(d) for d in dirs if d and os.path.isdir(d)]
    tail = [os.path.abspath(x) for x in os.environ.get(env_key, "").split(sep) if x.strip()]
    merged: list[str] = []
    for d in head + tail:
        if d not in merged:
            merged.append(d)
    if merged:
        os.environ[env_key] = sep.join(merged)
    return head


def _parse_runpath(readelf_stdout: str) -> str | None:
    for pattern in (
        r"Library runpath:\s*\[([^\]]*)\]",
        r"Library rpath:\s*\[([^\]]*)\]",
    ):
        m = re.search(pattern, readelf_stdout, flags=re.IGNORECASE)
        if m:
            return m.group(1).strip()
    return None


def _runpath_entry_exists(entry: str, crp_py_so: str) -> bool:
    entry = entry.strip()
    if not entry:
        return False
    if entry.startswith("$ORIGIN"):
        suffix = entry[len("$ORIGIN") :].lstrip("/\\")
        origin = os.path.dirname(os.path.abspath(crp_py_so))
        check = os.path.join(origin, suffix) if suffix else origin
        return os.path.isdir(check)
    return os.path.isdir(entry)


def _maybe_patch_runpath(crp_py_so: str) -> None:
    global _crp_runpath_state

    if not sys.platform.startswith("linux"):
        return

    crp_py_so = os.path.abspath(crp_py_so)
    state = _crp_runpath_state.get(crp_py_so)
    if state in ("ok", "patched") or not os.path.isfile(crp_py_so):
        if os.path.isfile(crp_py_so):
            _crp_runpath_state.setdefault(crp_py_so, "ok")
        return

    readelf = shutil.which("readelf")
    if not readelf:
        return

    try:
        proc = subprocess.run(
            [readelf, "-d", crp_py_so],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return

    raw = _parse_runpath(proc.stdout or "")
    if raw is None:
        _crp_runpath_state[crp_py_so] = "ok"
        return

    parts = [p for p in raw.split(":") if p.strip()]
    if parts and all(_runpath_entry_exists(p, crp_py_so) for p in parts):
        _crp_runpath_state[crp_py_so] = "ok"
        return

    patchelf = shutil.which("patchelf")
    if not patchelf:
        if state != "bad":
            logger.warning(
                "CrpRobotPy.so RUNPATH %r is invalid; install patchelf or run: "
                "patchelf --set-rpath '$ORIGIN' %s",
                raw,
                crp_py_so,
            )
            _crp_runpath_state[crp_py_so] = "bad"
        return

    try:
        pr = subprocess.run(
            [patchelf, "--set-rpath", "$ORIGIN", crp_py_so],
            capture_output=True,
            text=True,
            timeout=30,
            check=False,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        logger.warning("patchelf failed: %s", exc)
        return

    if pr.returncode != 0:
        logger.warning(
            "patchelf exit %s: %s",
            pr.returncode,
            (pr.stderr or pr.stdout or "").strip(),
        )
        return

    _crp_runpath_state[crp_py_so] = "patched"
    logger.info("Patched CrpRobotPy.so RUNPATH to $ORIGIN.")


def ensure_crp_sdk_ld_path(third_party_root: str | Path | None = None) -> list[str]:
    """Prepend SDK directory to the dynamic linker search path (idempotent)."""
    sdk_dir = _crp_sdk_dir(Path(third_party_root) if third_party_root else None)
    dirs = [str(sdk_dir)] if sdk_dir.is_dir() else []

    try:
        import CrpRobotPy  # noqa: PLC0415

        ext_dir = os.path.dirname(os.path.abspath(CrpRobotPy.__file__))
        if os.path.isdir(ext_dir) and ext_dir not in dirs:
            dirs.append(ext_dir)
    except Exception:
        pass

    return _prepend_env_lib_path(dirs)


def _is_vendor_pybind_method(method: object | None) -> bool:
    if method is None:
        return False
    qual = getattr(method, "__qualname__", "") or ""
    return "pybind11" in qual


def _vendor_has_native_dual_api(cls: type) -> bool:
    required = (
        "read_end_pose_user_second",
        "read_joints_second",
        "set_ui_first",
        "set_ui_second",
        "get_ui_first",
        "get_ui_second",
    )
    return all(_is_vendor_pybind_method(getattr(cls, name, None)) for name in required)


def install_crp_dual_patch(sdk_dir: Path | None = None) -> bool:
    """Attach legacy dual-arm helpers via CrpRobotPatch.so when native API is missing."""
    global _dual_patch_installed
    if _dual_patch_installed:
        return True

    root = Path(sdk_dir).resolve() if sdk_dir is not None else _crp_sdk_dir()

    try:
        import CrpRobotPy  # noqa: PLC0415
    except ImportError as exc:
        logger.warning("CrpRobotPy import failed: %s", exc)
        return False

    cls = CrpRobotPy.CrpRobotPy
    if _vendor_has_native_dual_api(cls):
        _dual_patch_installed = True
        register_vendor_dual_process_exit()
        return True

    patch_so = root / "CrpRobotPatch.so"
    if not patch_so.is_file():
        logger.warning(
            "CrpRobotPatch.so not found at %s; build: bash third_party/CrpRobotPy/build_patch.sh",
            patch_so,
        )
        return False

    try:
        import CrpRobotPatch  # noqa: PLC0415
    except ImportError as exc:
        logger.warning("CrpRobotPatch import failed: %s", exc)
        return False

    CrpRobotPatch.set_sdk_directory(str(root))

    def _read_end_pose_user_second(self):  # noqa: ANN001
        return CrpRobotPatch.read_end_pose_user_second(self)

    def _read_joints_second(self):  # noqa: ANN001
        joints = CrpRobotPatch.read_joints_second(self)
        if len(joints) != 6:
            raise RuntimeError(f"read_joints_second returned len {len(joints)}, expected 6")
        return {f"j{i + 1}": float(joints[i]) for i in range(6)}

    def _set_ui_first(self, index, value):  # noqa: ANN001
        CrpRobotPatch.set_ui_first(self, int(index), int(value))

    def _set_ui_second(self, index, value):  # noqa: ANN001
        CrpRobotPatch.set_ui_second(self, int(index), int(value))

    def _get_ui_first(self, index):  # noqa: ANN001
        return CrpRobotPatch.get_ui_first(self, int(index))

    def _get_ui_second(self, index):  # noqa: ANN001
        return CrpRobotPatch.get_ui_second(self, int(index))

    def _get_ui_count_first(self):  # noqa: ANN001
        return CrpRobotPatch.get_ui_count_first(self)

    def _get_ui_count_second(self):  # noqa: ANN001
        return CrpRobotPatch.get_ui_count_second(self)

    cls.read_end_pose_user_second = _read_end_pose_user_second
    cls.read_joints_second = _read_joints_second
    cls.set_ui_first = _set_ui_first
    cls.set_ui_second = _set_ui_second

    for name, fn in (
        ("get_ui_count_first", _get_ui_count_first),
        ("get_ui_count_second", _get_ui_count_second),
        ("get_ui_first", _get_ui_first),
        ("get_ui_second", _get_ui_second),
    ):
        if not _is_vendor_pybind_method(getattr(cls, name, None)):
            setattr(cls, name, fn)

    _dual_patch_installed = True
    logger.info("CrpRobotPy: legacy dual-arm patch installed")
    return True


def _repo_root_from_sdk(sdk_dir: Path) -> Path | None:
    for parent in sdk_dir.resolve().parents:
        if (parent / "third_party" / "CrpRobotPy").is_dir():
            return parent
    return None


def ensure_crp_license_key(sdk_dir: Path | None = None) -> Path | None:
    """Copy ``license.key`` next to ``libRobotService.so`` (SDK fails with unlicensed otherwise)."""
    root_dir = _crp_sdk_dir() if sdk_dir is None else Path(sdk_dir).resolve()
    dest = root_dir / "license.key"

    repo = _repo_root_from_sdk(root_dir)
    candidates: list[Path] = []
    if repo is not None:
        candidates.append(repo / "license.key")
    candidates.append(root_dir / "license.key")
    candidates.append(root_dir.parent / "license.key")

    src: Path | None = None
    for candidate in candidates:
        if candidate.is_file():
            src = candidate
            break

    if src is not None and (not dest.is_file() or src.read_bytes() != dest.read_bytes()):
        shutil.copy2(src, dest)
        logger.info("Installed CRP license.key at %s (from %s)", dest, src)
    elif dest.is_file():
        return dest

    if not dest.is_file():
        logger.warning(
            "CRP license.key missing at %s — SDK connect will fail with unlicensed. "
            "Copy ~/lerobot/license.key (vendor) next to libRobotService.so.",
            dest,
        )
        return None

    lib = root_dir / "libRobotService.so"
    if lib.is_file():
        # Vendor lib (~14.4MB) + repo license.key; OSS lib (~13.5MB) needs OSS license.
        size_mb = lib.stat().st_size / (1024 * 1024)
        if size_mb < 14.0 and dest.is_file():
            logger.warning(
                "libRobotService.so looks like CrobotpOSSDK OSS build (%.1f MB) but license.key "
                "is typically for the vendor library (~14.4 MB). Restore vendor "
                "libRobotService.so from lerobot_old/third_party/CrpRobotPy or rebuild "
                "without overwriting (see CrpRobotPy build.sh).",
                size_mb,
            )
    return dest


def _sync_license_next_to_module(module_file: str) -> None:
    """If CrpRobotPy loads from outside third_party, mirror license.key there too."""
    mod_dir = Path(module_file).resolve().parent
    sdk_dir = _crp_sdk_dir()
    src = sdk_dir / "license.key"
    if not src.is_file():
        ensure_crp_license_key(sdk_dir)
        src = sdk_dir / "license.key"
    if not src.is_file() or mod_dir == sdk_dir.resolve():
        return
    dest = mod_dir / "license.key"
    if not dest.is_file() or src.read_bytes() != dest.read_bytes():
        shutil.copy2(src, dest)
        logger.info("Installed CRP license.key at %s (mirror for loaded module)", dest)


def load_CrpRobotPy(third_party_root: str | Path | None = None) -> Path:
    """Add CrpRobotPy to sys.path, fix RUNPATH, preload libRobotService, install dual patch."""
    sdk_dir = _crp_sdk_dir(Path(third_party_root) if third_party_root else None)
    crp_py_so = sdk_dir / "CrpRobotPy.so"

    _maybe_patch_runpath(str(crp_py_so))
    ensure_crp_license_key(sdk_dir)

    pkg = str(sdk_dir)
    if pkg not in sys.path:
        sys.path.insert(0, pkg)

    ensure_crp_sdk_ld_path(sdk_dir.parent)

    service_so = sdk_dir / "libRobotService.so"
    if service_so.is_file():
        try:
            ctypes.CDLL(str(service_so.resolve()))
        except OSError:
            pass

    install_crp_dual_patch(sdk_dir)

    try:
        import CrpRobotPy  # noqa: PLC0415

        _sync_license_next_to_module(CrpRobotPy.__file__)
    except ImportError:
        pass

    return sdk_dir


def ensure_crp_sdk_loaded() -> None:
    """Load third_party/CrpRobotPy once (idempotent)."""
    global _sdk_loaded
    if _sdk_loaded:
        return
    load_CrpRobotPy()
    _sdk_loaded = True
