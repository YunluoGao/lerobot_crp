// Layout of Crp::CrpRobot in CrpRobotPy.so (reverse-engineered from ELF + live probe).
// Used only by CrpRobotPatch; must match the vendor binary ABI.

#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace Crp {

struct SRobotPosture {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};

// Byte offsets verified on hardware (2026-06): value_ptr memory dump after dual connect.
constexpr std::size_t kOffPrimaryLoader = 0x00;
constexpr std::size_t kOffIoPrimary = 0x18;
constexpr std::size_t kOffDualSession = 0x20;
constexpr std::size_t kOffServicePrimary = 0x28;
constexpr std::size_t kOffServiceSecond = 0x30;
constexpr std::size_t kOffField38 = 0x38;
constexpr std::size_t kOffServiceSecondAlt = 0x40;
constexpr std::size_t kOffSecondSessionReady = 0x50;

struct CrpRobotLayout {
    std::uint8_t _raw[0x58];
};

inline void *layout_ptr(void *robot_this, std::size_t offset) {
    return *reinterpret_cast<void **>(static_cast<char *>(robot_this) + offset);
}

inline void *primary_loader(void *robot_this) {
    return layout_ptr(robot_this, kOffPrimaryLoader);
}
inline void *io_primary(void *robot_this) {
    return layout_ptr(robot_this, kOffIoPrimary);
}
inline void *dual_session(void *robot_this) {
    return layout_ptr(robot_this, kOffDualSession);
}
inline void *service_primary(void *robot_this) {
    return layout_ptr(robot_this, kOffServicePrimary);
}
inline void *service_second(void *robot_this) {
    return layout_ptr(robot_this, kOffServiceSecond);
}
inline void *field_38(void *robot_this) {
    return layout_ptr(robot_this, kOffField38);
}
inline void *service_second_alt(void *robot_this) {
    return layout_ptr(robot_this, kOffServiceSecondAlt);
}
inline std::uint8_t second_session_ready(void *robot_this) {
    return *reinterpret_cast<std::uint8_t *>(static_cast<char *>(robot_this) + kOffSecondSessionReady);
}

inline CrpRobotLayout *as_layout(void *robot_this) {
    return reinterpret_cast<CrpRobotLayout *>(robot_this);
}

void set_sdk_directory(const char *sdk_dir);

std::vector<double> read_end_pose_user_second_impl(void *robot_this);
std::vector<double> read_joints_second_impl(void *robot_this);

int get_ui_count_primary(void *robot_this);
int get_ui_count_second(void *robot_this);
bool set_ui_primary(void *robot_this, std::size_t index, short value);
bool set_ui_second(void *robot_this, std::size_t index, short value);
bool get_ui_primary(void *robot_this, std::size_t index, short *out);
bool get_ui_second(void *robot_this, std::size_t index, short *out);

} // namespace Crp
