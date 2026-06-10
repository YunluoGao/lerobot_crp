///
/// Official CrobotpOSSDK IRobotService::getUI reader (not CrpRobotPy / ShowUI).
/// Used as a subprocess by lerobot-crp_tele_dual for gripper UI50/56–58 heartbeat logs.
/// Standalone CLI is for build verification and diagnostics (see README.md).
///

#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "CSDKLoader.h"
#include "IRobotService.h"

namespace {

constexpr char const *kRobotServiceDll = ROBOT_SERVICE_NAME;

volatile sig_atomic_t g_stopping = 0;

void signal_handler(int signum) {
  (void)signum;
  g_stopping = 1;
}

[[noreturn]] void fail_exit(int code) {
  std::fflush(stdout);
  std::fflush(stderr);
  // CSDKLoader destructor can segfault after failed connect(); skip teardown.
  _exit(code);
}

void print_usage(const char *prog) {
  std::fprintf(
      stderr,
      "Usage: %s <robot_ip> [options]\n"
      "\n"
      "crp_getui_probe — IRobotService::getUI only (read). Subprocess of lerobot-crp_tele_dual.\n"
      "Run from the directory that contains libRobotService.so, or set LD_LIBRARY_PATH.\n"
      "\n"
      "Options:\n"
      "  --indices LIST     Comma-separated UI indices (default: 50,56,57,58)\n"
      "  --hz FLOAT         Poll rate in Hz (default: 2.0, max 200)\n"
      "  --count N          Stop after N samples (default: infinite until Ctrl+C)\n"
      "  --json             One JSON object per line\n"
      "  --no-clear-error   Do not call clearError() after connect\n"
      "  --connect-retries N  Call connect() up to N times (default: 1; >1 may upset SDK)\n"
      "  --arm-label NAME   Include \"arm\" in JSON output (e.g. left, right)\n"
      "  -h, --help         Show this help\n"
      "\n"
      "Examples:\n"
      "  %s 192.168.0.100 --count 1\n"
      "  %s 192.168.0.101 --json --arm-label right\n"
      "  %s 192.168.0.100 --json --arm-label left --hz 5 > ui_left.jsonl\n",
      prog,
      prog,
      prog,
      prog);
}

bool parse_indices(const char *text, std::vector<size_t> *out) {
  out->clear();
  if (text == nullptr || text[0] == '\0') {
    return false;
  }
  const char *p = text;
  while (*p != '\0') {
    char *end = nullptr;
    const unsigned long v = std::strtoul(p, &end, 10);
    if (end == p) {
      return false;
    }
    out->push_back(static_cast<size_t>(v));
    if (*end == ',') {
      p = end + 1;
      continue;
    }
    if (*end == '\0') {
      break;
    }
    return false;
  }
  return !out->empty();
}

double now_seconds() {
  using clock = std::chrono::steady_clock;
  const auto t = clock::now().time_since_epoch();
  return std::chrono::duration<double>(t).count();
}

}  // namespace

int main(int argc, char *argv[]) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 2;
  }

  const char *robot_ip = nullptr;
  std::vector<size_t> indices = {50, 56, 57, 58};
  double hz = 2.0;
  long long max_samples = -1;
  bool json_output = false;
  bool clear_error = true;
  int connect_retries = 1;
  const char *arm_label = nullptr;

  for (int i = 1; i < argc; ++i) {
    const char *arg = argv[i];
    if (std::strcmp(arg, "-h") == 0 || std::strcmp(arg, "--help") == 0) {
      print_usage(argv[0]);
      return 0;
    }
    if (std::strcmp(arg, "--json") == 0) {
      json_output = true;
      continue;
    }
    if (std::strcmp(arg, "--no-clear-error") == 0) {
      clear_error = false;
      continue;
    }
    if (std::strcmp(arg, "--indices") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --indices requires a value\n");
        return 2;
      }
      if (!parse_indices(argv[++i], &indices)) {
        std::fprintf(stderr, "error: invalid --indices %s\n", argv[i]);
        return 2;
      }
      continue;
    }
    if (std::strcmp(arg, "--hz") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --hz requires a value\n");
        return 2;
      }
      hz = std::atof(argv[++i]);
      if (hz <= 0.0 || hz > 200.0) {
        std::fprintf(stderr, "error: --hz must be in (0, 200]\n");
        return 2;
      }
      continue;
    }
    if (std::strcmp(arg, "--count") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --count requires a value\n");
        return 2;
      }
      max_samples = std::atoll(argv[++i]);
      if (max_samples < 1) {
        std::fprintf(stderr, "error: --count must be >= 1\n");
        return 2;
      }
      continue;
    }
    if (std::strcmp(arg, "--connect-retries") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --connect-retries requires a value\n");
        return 2;
      }
      connect_retries = std::atoi(argv[++i]);
      if (connect_retries < 1) {
        std::fprintf(stderr, "error: --connect-retries must be >= 1\n");
        return 2;
      }
      continue;
    }
    if (std::strcmp(arg, "--arm-label") == 0) {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "error: --arm-label requires a value\n");
        return 2;
      }
      arm_label = argv[++i];
      continue;
    }
    if (arg[0] == '-') {
      std::fprintf(stderr, "error: unknown option %s\n", arg);
      print_usage(argv[0]);
      return 2;
    }
    if (robot_ip != nullptr) {
      std::fprintf(stderr, "error: unexpected argument %s\n", arg);
      print_usage(argv[0]);
      return 2;
    }
    robot_ip = arg;
  }

  if (robot_ip == nullptr) {
    std::fprintf(stderr, "error: missing <robot_ip>\n");
    print_usage(argv[0]);
    return 2;
  }

  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);

  Crp::CSDKLoader loader(kRobotServiceDll);

  if (!loader.initialize()) {
    std::fprintf(stderr, "error: CSDKLoader::initialize failed (is libRobotService.so on LD_LIBRARY_PATH?)\n");
    fail_exit(1);
  }

  Crp::IRobotService *robot = loader.getService<Crp::IRobotService>(ID_ROBOT_SERVICE);
  if (robot == nullptr) {
    std::fprintf(stderr, "error: getService<IRobotService> failed\n");
    std::fprintf(
        stderr,
        "hint: log shows 'unlicensed' => put license.key next to this binary (see README). "
        "Rebuild with: bash src/lerobot/robots/crp_arm_dual/getui_probe/build.sh\n");
    fail_exit(1);
  }

  bool connected = false;
  for (int attempt = 1; attempt <= connect_retries; ++attempt) {
    if (robot->connect(robot_ip)) {
      connected = true;
      break;
    }
    if (!json_output) {
      std::fprintf(stderr, "connect attempt %d/%d failed\n", attempt, connect_retries);
    }
  }
  if (!connected) {
    std::fprintf(stderr, "error: connect(%s) failed after %d attempt(s)\n", robot_ip, connect_retries);
    std::fprintf(
        stderr,
        "hint: ping %s; power/servo on; stop lerobot-crp_tele_dual if the cabinet allows only one SDK "
        "session; check firewall.\n",
        robot_ip);
    fail_exit(1);
  }

  if (!json_output) {
    std::printf("connected ip=%s sdk=%s\n", robot_ip, kRobotServiceDll);
    std::fflush(stdout);
  }

  if (robot->hasError()) {
    if (clear_error) {
      robot->clearError();
      if (!json_output) {
        std::printf("cleared controller error(s) after connect\n");
        std::fflush(stdout);
      }
    } else if (!json_output) {
      std::printf("warning: controller reports error(s); use --no-clear-error to skip auto clear\n");
      std::fflush(stdout);
    }
  }

  const int32_t ui_count = robot->getUICount();
  if (!json_output) {
    std::printf("getUICount=%d indices=", ui_count);
    for (size_t idx : indices) {
      std::printf("%zu ", idx);
    }
    std::printf("hz=%.3f\n", hz);
    std::fflush(stdout);
  }

  if (ui_count >= 0) {
    for (size_t idx : indices) {
      if (static_cast<int32_t>(idx) >= ui_count) {
        std::fprintf(
            stderr,
            "warning: UI index %zu >= getUICount()=%d (read may fail)\n",
            idx,
            ui_count);
      }
    }
  }

  const auto period = std::chrono::duration<double>(1.0 / hz);
  long long sample = 0;

  while (!g_stopping) {
    if (max_samples >= 0 && sample >= max_samples) {
      break;
    }
    if (robot->hasError()) {
      std::fprintf(stderr, "error: controller hasError() before sample %lld — stopping\n", sample);
      break;
    }

    const double t = now_seconds();
    bool any_fail = false;

    if (json_output) {
      std::printf("{\"t\":%.6f,\"ip\":\"%s\",\"sample\":%lld", t, robot_ip, sample);
      if (arm_label != nullptr && arm_label[0] != '\0') {
        std::printf(",\"arm\":\"%s\"", arm_label);
      }
    } else {
      std::printf("[%.3f] sample=%lld", t, sample);
    }

    for (size_t idx : indices) {
      int16_t value = 0;
      const bool ok = robot->getUI(idx, value);
      if (!ok) {
        any_fail = true;
      }
      if (json_output) {
        std::printf(",\"ui%zu_ok\":%s,\"ui%zu\":%d", idx, ok ? "true" : "false", idx, static_cast<int>(value));
      } else {
        std::printf(" UI%zu=%s%d", idx, ok ? "" : "FAIL:", static_cast<int>(value));
      }
    }

    if (json_output) {
      std::printf("}\n");
    } else {
      std::printf("\n");
    }
    std::fflush(stdout);

    if (any_fail && !json_output) {
      std::fprintf(stderr, "warning: one or more getUI calls returned false\n");
    }

    ++sample;
    std::this_thread::sleep_for(period);
  }

  if (robot->isConnected()) {
    robot->disconnect();
  }

  if (!json_output) {
    std::printf("disconnected ip=%s samples=%lld\n", robot_ip, sample);
    std::fflush(stdout);
  }

  return 0;
}
