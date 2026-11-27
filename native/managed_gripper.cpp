// Managed counterpart to the Polymetis franka_hand_client. The legacy client is unchanged.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <future>
#include <stdexcept>
#include <string>
#include <system_error>

#include <franka/gripper.h>
#include <grpcpp/grpcpp.h>
#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "polymetis.grpc.pb.h"
#include "polymetis/utils.h"

namespace {

constexpr int kReportedGripperHz = 30;
constexpr auto kRpcTimeout = std::chrono::seconds(1);
static_assert(ATOMIC_BOOL_LOCK_FREE == 2, "Signal handling requires a lock-free atomic flag");
std::atomic<bool> stop_requested{false};

void requestStop(int) noexcept {
  stop_requested.store(true, std::memory_order_relaxed);
}

bool stopping() {
  return stop_requested.load(std::memory_order_relaxed);
}

void installSignalHandlers() {
  struct sigaction action {};
  action.sa_handler = requestStop;
  action.sa_flags = SA_RESTART;
  sigemptyset(&action.sa_mask);
  if (sigaction(SIGINT, &action, nullptr) == -1 || sigaction(SIGTERM, &action, nullptr) == -1) {
    throw std::system_error(errno, std::generic_category(), "Could not install shutdown handlers");
  }
}

void setRpcDeadline(grpc::ClientContext& context) {
  context.set_deadline(std::chrono::system_clock::now() + kRpcTimeout);
}

void requireRpcSuccess(const grpc::Status& status, const char* operation) {
  if (!status.ok()) {
    throw std::runtime_error(std::string(operation) + " failed: " + status.error_message());
  }
}

bool applyCommand(franka::Gripper& gripper, const GripperCommand& command) {
  if (stopping()) {
    return false;
  }
  const auto width = command.width();
  const auto speed = command.speed();
  const bool grasp = command.grasp();
  spdlog::info("{} at width {} at speed={}", grasp ? "Grasping" : "Moving", width, speed);
  if (stopping()) {
    return false;
  }
  if (grasp) {
    // Keep the deployed client's positional arguments exactly as-is.
    return gripper.grasp(width, speed, 0.1, 0.1, command.force());
  }
  return gripper.move(width, speed);
}

void run(const YAML::Node& config) {
  const auto robot_ip = config["robot_ip"].as<std::string>();
  const auto control_address =
      config["control_ip"].as<std::string>() + ":" + config["control_port"].as<std::string>();
  auto stub =
      GripperServer::NewStub(grpc::CreateChannel(control_address, grpc::InsecureChannelCredentials()));

  spdlog::info("Connecting to Franka Hand at {}", robot_ip);
  if (stopping()) {
    return;
  }
  franka::Gripper gripper(robot_ip);
  if (stopping()) {
    return;
  }
  if (!gripper.homing()) {
    throw std::runtime_error("Franka Hand homing did not complete successfully");
  }
  if (stopping()) {
    return;
  }
  const auto initial_state = gripper.readOnce();
  if (stopping()) {
    return;
  }

  GripperMetadata metadata;
  metadata.set_max_width(initial_state.max_width);
  metadata.set_hz(kReportedGripperHz);
  grpc::ClientContext init_context;
  setRpcDeadline(init_context);
  Empty empty;
  requireRpcSuccess(stub->InitRobotClient(&init_context, metadata, &empty), "Gripper registration");
  if (!stopping()) {
    spdlog::info("Franka Hand connected.");
  }

  // An async future waits on destruction, also when an SDK or RPC call throws.
  // Declare it after the hand so that it always drains before libfranka disconnects.
  std::future<bool> pending;
  bool previous_success = true;
  int previous_timestamp_ns = 0;
  float previous_width = static_cast<float>(initial_state.width);

  // The deployed client polls at its SDK/RPC cadence. Its 30 Hz value is metadata.
  while (!stopping()) {
    const auto sample = gripper.readOnce();
    if (stopping()) {
      break;
    }
    if (pending.valid() &&
        pending.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      previous_success = pending.get();
    }
    GripperState state;
    state.set_width(sample.width);
    state.set_is_grasped(sample.is_grasped);
    state.set_is_moving(pending.valid());
    state.set_prev_command_successful(previous_success);
    setTimestampToNow(state.mutable_timestamp());

    grpc::ClientContext context;
    setRpcDeadline(context);
    GripperCommand command;
    requireRpcSuccess(stub->ControlUpdate(&context, state, &command), "Gripper control update");
    if (stopping()) {
      break;
    }

    const int timestamp_ns = command.timestamp().nanos();
    const float width = static_cast<float>(command.width());
    if (!pending.valid() && timestamp_ns != 0 && timestamp_ns != previous_timestamp_ns &&
        std::fabs(width - previous_width) > 0.045) {
      // libfranka permits reads during a command. The worker owns its command snapshot.
      pending = std::async(std::launch::async, [&gripper, command] {
        return applyCommand(gripper, command);
      });
      previous_timestamp_ns = timestamp_ns;
      previous_width = width;
    }
  }

  if (pending.valid()) {
    // Report SDK exceptions normally, even when shutdown is already in progress.
    pending.get();
  }
}

}  // namespace

int main(int argc, char* argv[]) {
  if (argc != 2) {
    spdlog::error("Usage: managed_gripper /path/to/cfg.yaml");
    return 1;
  }

  try {
    installSignalHandlers();
    run(YAML::LoadFile(argv[1]));
    spdlog::info("Managed Franka Hand shutdown complete.");
    return 0;
  } catch (const std::exception& error) {
    spdlog::error("Managed Franka Hand client failed: {}", error.what());
    return 1;
  }
}
