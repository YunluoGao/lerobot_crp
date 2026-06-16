#include "crp_robot_layout.hpp"

#include <pybind11/detail/class.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>

namespace py = pybind11;

namespace {

void *crp_robot_this_ptr(py::handle obj) {
    if (!obj) {
        throw std::runtime_error("expected CrpRobotPy instance");
    }
    auto *inst = reinterpret_cast<pybind11::detail::instance *>(obj.ptr());
    if (inst == nullptr) {
        throw std::runtime_error("not a pybind11 instance");
    }

    // simple_layout: value lives in simple_value_holder[0] and equals value_ptr().
    if (inst->simple_layout && inst->simple_value_holder[0] != nullptr) {
        return inst->simple_value_holder[0];
    }

    pybind11::detail::values_and_holders vhs(inst);
    if (vhs.size() > 0) {
        void *value_ptr = vhs.begin()->value_ptr();
        if (value_ptr != nullptr) {
            return value_ptr;
        }
    }

    throw std::runtime_error(
        "cannot read CrpRobot C++ pointer (unexpected holder layout); "
        "rebuild CrpRobotPatch or update crp_robot_layout.hpp");
}

} // namespace

PYBIND11_MODULE(CrpRobotPatch, m) {
    m.doc() = "Patch for dual-arm CrpRobotPy: second TCP/joints read + UI read/write";

    m.def(
        "set_sdk_directory",
        [](const std::string &sdk_dir) { Crp::set_sdk_directory(sdk_dir.c_str()); },
        py::arg("sdk_dir"),
        "Directory containing CrpRobotPy.so and libRobotService.so (for dlsym).");

    m.def(
        "read_end_pose_user_second",
        [](py::object robot) { return Crp::read_end_pose_user_second_impl(crp_robot_this_ptr(robot)); },
        py::arg("robot"),
        "Read user-frame TCP (mm, deg) from second arm (ip2 / connect_second).");

    m.def(
        "read_joints_second",
        [](py::object robot) { return Crp::read_joints_second_impl(crp_robot_this_ptr(robot)); },
        py::arg("robot"),
        "Read body joint angles (deg) from second arm; same order as read_joints() j1..j6.");

    m.def(
        "get_ui_count_first",
        [](py::object robot) { return Crp::get_ui_count_primary(crp_robot_this_ptr(robot)); },
        py::arg("robot"));

    m.def(
        "get_ui_count_second",
        [](py::object robot) { return Crp::get_ui_count_second(crp_robot_this_ptr(robot)); },
        py::arg("robot"));

    m.def(
        "set_ui_first",
        [](py::object robot, std::size_t index, int value) {
            return Crp::set_ui_primary(crp_robot_this_ptr(robot), index, static_cast<short>(value));
        },
        py::arg("robot"),
        py::arg("index"),
        py::arg("value"));

    m.def(
        "set_ui_second",
        [](py::object robot, std::size_t index, int value) {
            return Crp::set_ui_second(crp_robot_this_ptr(robot), index, static_cast<short>(value));
        },
        py::arg("robot"),
        py::arg("index"),
        py::arg("value"));

    m.def(
        "get_ui_first",
        [](py::object robot, std::size_t index) {
            short value = 0;
            if (!Crp::get_ui_primary(crp_robot_this_ptr(robot), index, &value)) {
                throw std::runtime_error("getUI failed on primary arm");
            }
            return static_cast<int>(value);
        },
        py::arg("robot"),
        py::arg("index"));

    m.def(
        "get_ui_second",
        [](py::object robot, std::size_t index) {
            short value = 0;
            if (!Crp::get_ui_second(crp_robot_this_ptr(robot), index, &value)) {
                throw std::runtime_error("getUI failed on second arm");
            }
            return static_cast<int>(value);
        },
        py::arg("robot"),
        py::arg("index"));
}
