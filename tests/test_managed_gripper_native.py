"""Exercise the native client executable against inert SDK and gRPC stand-ins."""

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path

import pytest

_FAKE_EVENTS = r"""
#pragma once
#include <cstdlib>
#include <fstream>
#include <string>

inline std::string scenario() { return std::getenv("TEST_SCENARIO"); }
inline void record(const std::string& event) {
  std::ofstream(std::getenv("TEST_EVENTS"), std::ios::app) << event << '\n';
}
"""

_FAKE_GRIPPER = r"""
#pragma once
#include "fake_events.h"
#include <chrono>
#include <csignal>
#include <stdexcept>
#include <thread>

namespace franka {
struct GripperState {
  double width = 0.0;
  double max_width = 0.08;
  bool is_grasped = false;
};

class Gripper {
 public:
  explicit Gripper(const std::string&) {
    record("connect");
    if (scenario() == "signal_during_connect") std::raise(SIGTERM);
  }

  ~Gripper() { record("disconnect"); }

  bool homing() {
    record("home");
    if (scenario() == "signal_during_homing") std::raise(SIGINT);
    if (scenario() == "homing_exception") throw std::runtime_error("fake homing error");
    return scenario() != "homing_false";
  }

  GripperState readOnce() { return {}; }

  bool move(double width, double speed) {
    if (width != 0.0625 || speed != 0.125) throw std::runtime_error("unexpected move args");
    return operate("move");
  }

  bool grasp(double width, double speed, double a, double b, double c) {
    if (width != 0.0625 || speed != 0.125 || a != 0.1 || b != 0.1 || c != 0.25) {
      throw std::runtime_error("unexpected grasp args");
    }
    return operate("grasp");
  }

  bool stop() {
    record("unexpected stop");
    return false;
  }

 private:
  bool operate(const std::string& command) {
    record(command + " started");
    if (scenario() == "command_exception") throw std::runtime_error("fake command error");
    const auto wait_limit = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (!std::ifstream(std::getenv("TEST_RELEASE"))) {
      if (std::chrono::steady_clock::now() > wait_limit) {
        throw std::runtime_error("test did not release command");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (scenario() == "command_exception_after_release") {
      record(command + " failed");
      throw std::runtime_error("fake in-flight command error");
    }
    record(command + " finished");
    return true;
  }
};
}  // namespace franka
"""

_FAKE_GRPC = r"""
#pragma once
#include "fake_events.h"
#include <chrono>
#include <memory>

namespace grpc {
class ClientContext {
 public:
  void set_deadline(std::chrono::system_clock::time_point deadline) {
    deadline_ = deadline;
  }

  bool bounded() const {
    const auto remaining = deadline_ - std::chrono::system_clock::now();
    return remaining > std::chrono::seconds(0) && remaining < std::chrono::seconds(5);
  }

 private:
  std::chrono::system_clock::time_point deadline_;
};

class Status {
 public:
  explicit Status(std::string message = "") : message_(message) {}
  bool ok() const { return message_.empty(); }
  std::string error_message() const { return message_; }

 private:
  std::string message_;
};

class Channel {};
inline int InsecureChannelCredentials() { return 0; }
inline std::shared_ptr<Channel> CreateChannel(const std::string&, int) {
  return std::make_shared<Channel>();
}
}  // namespace grpc
"""

_FAKE_PROTO = r"""
#pragma once
#include "grpcpp/grpcpp.h"
#include <csignal>

struct Empty {};
struct Timestamp { int nanos() const { return 1; } };

struct GripperCommand {
  bool grasp() const { return scenario() == "grasp"; }
  double width() const { return 0.0625; }
  double speed() const { return 0.125; }
  double force() const { return 0.25; }
  Timestamp timestamp() const { return {}; }
};

struct GripperMetadata {
  void set_max_width(double) {}
  void set_hz(int) {}
};

struct GripperState {
  void set_width(double) {}
  void set_is_grasped(bool) {}
  void set_is_moving(bool) {}
  void set_prev_command_successful(bool) {}
  Timestamp* mutable_timestamp() { return &timestamp; }
  Timestamp timestamp;
};

struct GripperServer {
  struct Stub {
    grpc::Status InitRobotClient(grpc::ClientContext* context, const GripperMetadata&, Empty*) {
      record(context->bounded() ? "register bounded" : "register unbounded");
      if (scenario() == "registration_failure") return grpc::Status("fake registration failure");
      return grpc::Status();
    }

    grpc::Status ControlUpdate(grpc::ClientContext* context, const GripperState&, GripperCommand*) {
      if (updates_++ == 0) {
        record(context->bounded() ? "update bounded" : "update unbounded");
      }
      if (scenario() == "signal_during_update") std::raise(SIGINT);
      if (scenario() == "update_failure") return grpc::Status("fake update failure");
      return grpc::Status();
    }

   private:
    unsigned int updates_ = 0;
  };

  static std::unique_ptr<Stub> NewStub(std::shared_ptr<grpc::Channel>) {
    return std::make_unique<Stub>();
  }
};
"""

_HEADERS = {
    "fake_events.h": _FAKE_EVENTS,
    "franka/gripper.h": _FAKE_GRIPPER,
    "franka/gripper_state.h": '#include "franka/gripper.h"\n',
    "grpc/grpc.h": '#include "grpcpp/grpcpp.h"\n',
    "grpcpp/grpcpp.h": _FAKE_GRPC,
    "polymetis.grpc.pb.h": _FAKE_PROTO,
    "polymetis/utils.h": r"""
#pragma once
#include "polymetis.grpc.pb.h"
inline void setTimestampToNow(Timestamp*) {}
""",
    "spdlog/spdlog.h": r"""
#pragma once
#include <iostream>
namespace spdlog {
template<class... Args> void info(const char*, const Args&...) {}
template<class... Args> void error(const char* message, const Args&... args) {
  std::cerr << message;
  using ignored = int[];
  (void)ignored{0, (std::cerr << " " << args, 0)...};
  std::cerr << '\n';
}
}
""",
    "yaml-cpp/yaml.h": r"""
#pragma once
#include <string>
namespace YAML {
struct Node {
  Node operator[](const char*) const { return {}; }
  template<class T> T as() const { return T("fake-address"); }
};
inline Node LoadFile(const char*) { return {}; }
}
""",
}


@pytest.fixture(scope="module")
def native_client(tmp_path_factory):
    compiler = shutil.which("c++")
    if not compiler:
        pytest.skip("a C++ compiler is required to test the native client")
    build = tmp_path_factory.mktemp("managed_gripper_native")
    for name, contents in _HEADERS.items():
        header = build / name
        header.parent.mkdir(parents=True, exist_ok=True)
        header.write_text(contents)
    native_source = Path(__file__).resolve().parents[1] / "native/managed_gripper.cpp"
    binary = build / "managed-gripper"
    subprocess.run(
        [
            compiler,
            "-std=c++14",
            "-pthread",
            "-I",
            str(build),
            str(native_source),
            "-o",
            str(binary),
        ],
        check=True,
        timeout=30,
    )
    return binary


def _fake_environment(tmp_path, scenario):
    return {
        **os.environ,
        "TEST_EVENTS": str(tmp_path / "events"),
        "TEST_RELEASE": str(tmp_path / "release"),
        "TEST_SCENARIO": scenario,
    }


def _events(tmp_path):
    path = tmp_path / "events"
    return path.read_text().splitlines() if path.exists() else []


@pytest.mark.parametrize(
    ("scenario", "command", "stop_signal", "return_code", "outcome"),
    [
        ("move", "move", signal.SIGINT, 0, "finished"),
        ("grasp", "grasp", signal.SIGTERM, 0, "finished"),
        ("command_exception_after_release", "move", signal.SIGTERM, 1, "failed"),
    ],
)
def test_signal_drains_in_flight_native_command_before_disconnect(
    native_client, tmp_path, scenario, command, stop_signal, return_code, outcome
):
    with subprocess.Popen(
        [str(native_client), "fake-config.yaml"],
        env=_fake_environment(tmp_path, scenario),
    ) as process:
        try:
            deadline = time.monotonic() + 3
            while f"{command} started" not in _events(tmp_path):
                assert process.poll() is None and time.monotonic() < deadline, _events(tmp_path)
                time.sleep(0.01)
            process.send_signal(stop_signal)
            with pytest.raises(subprocess.TimeoutExpired):
                process.wait(timeout=0.15)
            assert "disconnect" not in _events(tmp_path)
            (tmp_path / "release").touch()
            assert process.wait(timeout=3) == return_code
        finally:
            if process.poll() is None:
                process.kill()

    assert _events(tmp_path) == [
        "connect",
        "home",
        "register bounded",
        "update bounded",
        f"{command} started",
        f"{command} {outcome}",
        "disconnect",
    ]


@pytest.mark.parametrize(
    ("scenario", "expected"),
    [
        ("signal_during_connect", ["connect", "disconnect"]),
        ("signal_during_homing", ["connect", "home", "disconnect"]),
        (
            "signal_during_update",
            ["connect", "home", "register bounded", "update bounded", "disconnect"],
        ),
    ],
)
def test_signal_prevents_new_sdk_operations(native_client, tmp_path, scenario, expected):
    result = subprocess.run(
        [str(native_client), "fake-config.yaml"],
        env=_fake_environment(tmp_path, scenario),
        check=False,
        timeout=3,
    )

    assert result.returncode == 0
    assert _events(tmp_path) == expected


@pytest.mark.parametrize(
    ("scenario", "last_operation", "diagnostic"),
    [
        ("homing_false", "home", "homing"),
        ("homing_exception", "home", "fake homing error"),
        ("registration_failure", "register bounded", "fake registration failure"),
        ("update_failure", "update bounded", "fake update failure"),
        ("command_exception", "move started", "fake command error"),
    ],
)
def test_native_failure_releases_connection_and_exits_nonzero(
    native_client, tmp_path, scenario, last_operation, diagnostic
):
    result = subprocess.run(
        [str(native_client), "fake-config.yaml"],
        env=_fake_environment(tmp_path, scenario),
        capture_output=True,
        text=True,
        check=False,
        timeout=3,
    )

    assert result.returncode == 1
    assert diagnostic in result.stderr
    assert _events(tmp_path)[-2:] == [last_operation, "disconnect"]
