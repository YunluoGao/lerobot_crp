#include "crp_robot_layout.hpp"

#include <dlfcn.h>
#include <link.h>

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <string>
#include <stdexcept>
#include <vector>

namespace Crp {
namespace {

// Matches Crp::SJointPosition in RobotTypes.h
struct SJointPosition {
    double body[6];
    double ext[6];
    int cfg[4];
};

// IRobotService vtable slots (verified from CrpRobotPy.so disassembly).
constexpr std::size_t kVtableGetUserPosture = 0x40;
constexpr std::size_t kVtableGetWorldPosture = 0x48;
constexpr std::size_t kVtableGetCurrentJoint = 0x1f8;

// CrpRobot::ensureSecondSession (PIE offset in current vendor CrpRobotPy.so).
constexpr std::size_t kEnsureSecondSessionOffset = 0x15810;

using GetPostureFn = bool (*)(void *service_this, SRobotPosture *out);
using EnsureSecondSessionFn = void (*)(void *robot_this);
using GetCurrentJointFn = bool (*)(void *service_this, SJointPosition *out);

std::string g_sdk_dir;

void *open_sdk_library(const char *filename) {
    if (g_sdk_dir.empty()) {
        return dlopen(filename, RTLD_NOW | RTLD_GLOBAL);
    }
    const std::string path = g_sdk_dir + "/" + filename;
    void *handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (handle != nullptr) {
        return handle;
    }
    return dlopen(filename, RTLD_NOW | RTLD_GLOBAL);
}

void *resolve_symbol_ptr(const char *name) {
    static void *crp_py_handle = nullptr;
    static void *service_handle = nullptr;

    if (crp_py_handle == nullptr) {
        crp_py_handle = open_sdk_library("CrpRobotPy.so");
    }
    if (service_handle == nullptr) {
        service_handle = open_sdk_library("libRobotService.so");
    }

    void *sym = nullptr;
    if (crp_py_handle != nullptr) {
        sym = dlsym(crp_py_handle, name);
    }
    if (sym == nullptr && service_handle != nullptr) {
        sym = dlsym(service_handle, name);
    }
    if (sym == nullptr) {
        sym = dlsym(RTLD_DEFAULT, name);
    }
    if (sym == nullptr) {
        throw std::runtime_error(
            std::string("dlsym failed for ") + name + ": " + (dlerror() ? dlerror() : "unknown"));
    }
    return sym;
}

bool pointer_looks_valid(void *ptr) {
    if (ptr == nullptr) {
        return false;
    }
    const auto addr = reinterpret_cast<std::uintptr_t>(ptr);
    return addr >= 0x10000UL && (addr % alignof(void *)) == 0;
}

void *crp_robot_py_image_base() {
    static void *base = nullptr;
    if (base != nullptr) {
        return base;
    }
    void *handle = open_sdk_library("CrpRobotPy.so");
    if (handle == nullptr) {
        throw std::runtime_error("CrpRobotPy.so not loaded");
    }
    void *anchor = dlsym(handle, "PyInit_CrpRobotPy");
    if (anchor == nullptr) {
        throw std::runtime_error("PyInit_CrpRobotPy not found in CrpRobotPy.so");
    }
    Dl_info info{};
    if (dladdr(anchor, &info) == 0 || info.dli_fbase == nullptr) {
        throw std::runtime_error("dladdr failed for CrpRobotPy.so");
    }
    base = info.dli_fbase;
    return base;
}

EnsureSecondSessionFn resolve_ensure_second_session() {
    static EnsureSecondSessionFn fn = reinterpret_cast<EnsureSecondSessionFn>(
        static_cast<char *>(crp_robot_py_image_base()) + kEnsureSecondSessionOffset);
    return fn;
}

void ensure_second_session(void *robot_this) {
    resolve_ensure_second_session()(robot_this);
}

void ensure_second_session_if_needed(void *robot_this) {
    ensure_second_session(robot_this);
}

void *vtable_slot(void *service, std::size_t byte_offset) {
    if (!pointer_looks_valid(service)) {
        return nullptr;
    }
    void *vtable = *reinterpret_cast<void **>(service);
    if (!pointer_looks_valid(vtable)) {
        return nullptr;
    }
    return *reinterpret_cast<void **>(reinterpret_cast<std::uint8_t *>(vtable) + byte_offset);
}

bool call_vtable_posture(void *service, SRobotPosture *out, std::size_t vtable_offset) {
    void *slot = vtable_slot(service, vtable_offset);
    if (slot == nullptr) {
        return false;
    }
    auto fn = reinterpret_cast<GetPostureFn>(slot);
    return fn(service, out);
}

bool call_vtable_joints(void *service, SJointPosition *out) {
    void *slot = vtable_slot(service, kVtableGetCurrentJoint);
    if (slot == nullptr) {
        return false;
    }
    auto fn = reinterpret_cast<GetCurrentJointFn>(slot);
    return fn(service, out);
}

bool posture_is_effectively_zero(const SRobotPosture &p) {
    constexpr double eps = 1e-6;
    return std::abs(p.x) < eps && std::abs(p.y) < eps && std::abs(p.z) < eps && std::abs(p.rx) < eps
           && std::abs(p.ry) < eps && std::abs(p.rz) < eps;
}

bool read_posture_from_service(void *service, SRobotPosture *out) {
    *out = SRobotPosture{};
    if (call_vtable_posture(service, out, kVtableGetUserPosture)
        && !posture_is_effectively_zero(*out)) {
        return true;
    }
    *out = SRobotPosture{};
    return call_vtable_posture(service, out, kVtableGetWorldPosture)
           && !posture_is_effectively_zero(*out);
}

constexpr std::size_t kDualSessionDlopenOffset = 0x20;

void *second_loader_from_robot(void *robot_this) {
    void *session = dual_session(robot_this);
    if (pointer_looks_valid(session)) {
        void *loader = *reinterpret_cast<void **>(
            static_cast<char *>(session) + kDualSessionDlopenOffset);
        if (pointer_looks_valid(loader)) {
            return loader;
        }
    }
    return field_38(robot_this);
}

std::vector<void *> second_service_candidates(void *robot_this) {
    std::vector<void *> candidates;
    auto append = [&](void *svc) {
        if (!pointer_looks_valid(svc)) {
            return;
        }
        for (void *existing : candidates) {
            if (existing == svc) {
                return;
            }
        }
        candidates.push_back(svc);
    };
    append(service_second(robot_this));
    append(service_second_alt(robot_this));
    if (candidates.empty()) {
        throw std::runtime_error("second IRobotService is null (connect_second first)");
    }
    return candidates;
}

struct SecondArmBinding {
    void *service_second = nullptr;
    void *second_loader = nullptr;
};

SecondArmBinding find_second_arm_binding(void *robot_this) {
    if (robot_this == nullptr) {
        throw std::runtime_error("CrpRobot null pointer");
    }
    ensure_second_session_if_needed(robot_this);

    void *primary = service_primary(robot_this);
    void *service = service_second_alt(robot_this);
    if (!pointer_looks_valid(service) || service == primary) {
        service = service_second(robot_this);
    }
    if (!pointer_looks_valid(service)) {
        throw std::runtime_error("second arm IRobotService not found (service_second is null)");
    }
    return {service, second_loader_from_robot(robot_this)};
}

} // namespace

void set_sdk_directory(const char *sdk_dir) {
    g_sdk_dir = sdk_dir != nullptr ? sdk_dir : "";
    while (!g_sdk_dir.empty() && g_sdk_dir.back() == '/') {
        g_sdk_dir.pop_back();
    }
}

std::vector<double> read_end_pose_user_second_impl(void *robot_this) {
    if (robot_this == nullptr) {
        throw std::runtime_error("CrpRobot null pointer");
    }

    if (!pointer_looks_valid(service_second(robot_this))
        && !pointer_looks_valid(service_second_alt(robot_this))) {
        throw std::runtime_error("second arm not connected (connect_second first)");
    }

    ensure_second_session_if_needed(robot_this);

    for (void *service : second_service_candidates(robot_this)) {
        SRobotPosture trial{};
        if (!read_posture_from_service(service, &trial)) {
            continue;
        }
        return {trial.x, trial.y, trial.z, trial.rx, trial.ry, trial.rz};
    }

    throw std::runtime_error(
        "getUserPosture/getWorldPosture failed or returned zero on second arm "
        "(servo on + Auto mode?)");
}

std::vector<double> read_joints_second_impl(void *robot_this) {
    if (robot_this == nullptr) {
        throw std::runtime_error("CrpRobot null pointer");
    }

    if (!pointer_looks_valid(service_second(robot_this))
        && !pointer_looks_valid(service_second_alt(robot_this))) {
        throw std::runtime_error("second arm not connected (connect_second first)");
    }

    ensure_second_session_if_needed(robot_this);

    for (void *service : second_service_candidates(robot_this)) {
        SJointPosition pos{};
        if (call_vtable_joints(service, &pos)) {
            return {pos.body[0], pos.body[1], pos.body[2], pos.body[3], pos.body[4], pos.body[5]};
        }
    }

    throw std::runtime_error("getCurrentJoint failed on second arm (servo on?)");
}

// UI via CIOService (+0x18) -> embedded CRobotService (+0x20) -> __setInt<short>(type=6).
// Do NOT call CRobotService::setUI on IRobotService* (+0x08): that corrupts the heap.
constexpr int kVarShowUI = 6;
constexpr std::size_t kCIOServiceRobotSvcOffset = 0x20;
constexpr const char kIOServiceUuid[] = "EE2C547B-B554-4DD1-B9FE-EE84955EAC60";
constexpr const char kRobotServiceUuid[] = "A5236E6F-35E4-47C9-BAB1-1FC5E1DAED1B";
constexpr const char kGetIOServiceSym[] =
    "_ZN3Crp10CSDKLoader10getServiceINS_10IIOServiceEEEPT_PKc";
constexpr const char kGetRobotServiceSym[] =
    "_ZN3Crp10CSDKLoader10getServiceINS_13CRobotServiceEEEPT_PKc";
constexpr const char kSetIntShortSym[] =
    "_ZN3Crp13CRobotService8__setIntIsEEb17EVariableShowTypemPKT_m";
constexpr const char kGetIntShortSym[] =
    "_ZN3Crp13CRobotService8__getIntIsEEb17EVariableShowTypemPT_m";
constexpr const char kGetUICountSym[] = "_ZN3Crp13CRobotService10getUICountEv";

using GetIOServiceFn = void *(*)(void *loader, const char *name);
using SetIntShortFn = bool (*)(void *, int, unsigned long, const short *, unsigned long);
using GetIntShortFn = bool (*)(void *, int, unsigned long, short *, unsigned long);
using GetUICountFn = int (*)(void *);

void *robot_service_from_io(void *io_service) {
    if (io_service == nullptr) {
        throw std::runtime_error("IIOService is null");
    }
    auto *holder = reinterpret_cast<void **>(
        static_cast<char *>(io_service) + kCIOServiceRobotSvcOffset);
    void *robot_svc = holder != nullptr ? *holder : nullptr;
    if (robot_svc == nullptr) {
        throw std::runtime_error("CIOService embedded CRobotService is null");
    }
    return robot_svc;
}

void *resolve_io_second(void *robot_this) {
    const SecondArmBinding binding = find_second_arm_binding(robot_this);
    static void *cached_loader = nullptr;
    static void *cached_io = nullptr;
    if (binding.second_loader != cached_loader) {
        cached_loader = binding.second_loader;
        cached_io = nullptr;
    }
    if (cached_io != nullptr) {
        return cached_io;
    }
    if (!pointer_looks_valid(binding.second_loader)) {
        throw std::runtime_error("second_loader is null");
    }
    static GetIOServiceFn get_io =
        reinterpret_cast<GetIOServiceFn>(resolve_symbol_ptr(kGetIOServiceSym));
    cached_io = get_io(binding.second_loader, kIOServiceUuid);
    if (cached_io == nullptr) {
        throw std::runtime_error("getService<IIOService> failed on second arm");
    }
    return cached_io;
}

int get_ui_count_on_io(void *io_service) {
    static GetUICountFn get_count = reinterpret_cast<GetUICountFn>(resolve_symbol_ptr(kGetUICountSym));
    void *robot_svc = robot_service_from_io(io_service);
    return get_count(robot_svc);
}

bool set_ui_on_io(void *io_service, std::size_t index, short value) {
    const int count = get_ui_count_on_io(io_service);
    if (count >= 0 && static_cast<std::size_t>(index) >= static_cast<std::size_t>(count)) {
        return false;
    }
    static SetIntShortFn set_int = reinterpret_cast<SetIntShortFn>(resolve_symbol_ptr(kSetIntShortSym));
    void *robot_svc = robot_service_from_io(io_service);
    return set_int(robot_svc, kVarShowUI, static_cast<unsigned long>(index), &value, 1);
}

bool get_ui_on_io(void *io_service, std::size_t index, short *out) {
    if (out == nullptr) {
        return false;
    }
    const int count = get_ui_count_on_io(io_service);
    if (count >= 0 && static_cast<std::size_t>(index) >= static_cast<std::size_t>(count)) {
        return false;
    }
    static GetIntShortFn get_int = reinterpret_cast<GetIntShortFn>(resolve_symbol_ptr(kGetIntShortSym));
    void *robot_svc = robot_service_from_io(io_service);
    *out = 0;
    return get_int(robot_svc, kVarShowUI, static_cast<unsigned long>(index), out, 1);
}

int get_ui_count_primary(void *robot_this) {
    if (robot_this == nullptr) {
        return -1;
    }
    void *io = io_primary(robot_this);
    if (io == nullptr) {
        return -1;
    }
    return get_ui_count_on_io(io);
}

int get_ui_count_second(void *robot_this) {
    try {
        return get_ui_count_on_io(resolve_io_second(robot_this));
    } catch (...) {
        return -1;
    }
}

bool set_ui_primary(void *robot_this, std::size_t index, short value) {
    if (robot_this == nullptr) {
        throw std::runtime_error("CrpRobot null pointer");
    }
    void *io = io_primary(robot_this);
    if (io == nullptr) {
        throw std::runtime_error("IIOService missing on primary arm (connect first)");
    }
    if (!set_ui_on_io(io, index, value)) {
        throw std::runtime_error("setUI failed on primary arm");
    }
    return true;
}

bool set_ui_second(void *robot_this, std::size_t index, short value) {
    void *io_second = resolve_io_second(robot_this);
    if (!set_ui_on_io(io_second, index, value)) {
        throw std::runtime_error("setUI failed on second arm");
    }
    return true;
}

bool get_ui_primary(void *robot_this, std::size_t index, short *out) {
    if (robot_this == nullptr) {
        return false;
    }
    void *io = io_primary(robot_this);
    if (io == nullptr) {
        return false;
    }
    return get_ui_on_io(io, index, out);
}

bool get_ui_second(void *robot_this, std::size_t index, short *out) {
    try {
        return get_ui_on_io(resolve_io_second(robot_this), index, out);
    } catch (...) {
        return false;
    }
}

} // namespace Crp
