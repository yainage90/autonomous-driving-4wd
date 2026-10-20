// Copyright 2026 vehicle_team. Licensed under Apache-2.0.
#pragma once

#include <cmath>
#include <cstdint>
#include <map>
#include <limits>
#include <stdexcept>
#include <string>

#include <vehicle_interfaces/msg/actuator_command.hpp>
#include <vehicle_interfaces/msg/actuator_status.hpp>
#include <vehicle_interfaces/srv/arm_actuator.hpp>

namespace vehicle_actuator_sim
{
// Called under the plugin's mutex. Time is supplied by the receiver, never the sender.
class Watchdog
{
public:
  using Command = vehicle_interfaces::msg::ActuatorCommand;
  using Status = vehicle_interfaces::msg::ActuatorStatus;
  using Arm = vehicle_interfaces::srv::ArmActuator;

  Watchdog(std::string session, double timeout, double heartbeat_age,
    double output_period, double status_timeout, double max_linear, double max_angular)
  : timeout_ns_(0), heartbeat_age_ns_(0),
    max_linear_(max_linear), max_angular_(max_angular)
  {
    for (double value : {timeout, heartbeat_age, output_period, status_timeout,
      max_linear, max_angular})
      if (!std::isfinite(value) || value <= 0.0)
        throw std::invalid_argument("Actuator settings must be finite and positive");
    if (!(output_period < heartbeat_age && heartbeat_age < timeout &&
      output_period < status_timeout && status_timeout < timeout))
      throw std::invalid_argument("Require output period < freshness windows < watchdog");
    const double max_seconds = static_cast<double>(std::numeric_limits<int64_t>::max()) / 1e9;
    if (timeout >= max_seconds || heartbeat_age >= max_seconds)
      throw std::invalid_argument("Timing settings exceed the steady clock range");
    timeout_ns_ = static_cast<int64_t>(timeout * 1e9);
    heartbeat_age_ns_ = static_cast<int64_t>(heartbeat_age * 1e9);
    status.actuator_session = std::move(session);
    status.generation = 1;
    status.reason = "startup";
  }

  void tick(int64_t now)
  {
    if (status.armed && now - status.last_valid_ns >= timeout_ns_) {
      status.expired_ns = now;
      disarm("watchdog_expired");
    }
  }

  void disarm(const std::string & reason)
  {
    ++status.generation;
    status.armed = false;
    status.guard_session.clear();
    status.accepted_sequence = 0;
    status.reason = reason;
    status.zero_applied_ns = 0;
    linear = angular = 0.0;
    heartbeats_.clear();
  }

  Status heartbeat(int64_t now)
  {
    tick(now);
    ++status.heartbeat;
    status.steady_ns = now;
    heartbeats_[status.heartbeat] = now;
    while (!heartbeats_.empty() && now - heartbeats_.begin()->second >= heartbeat_age_ns_)
      heartbeats_.erase(heartbeats_.begin());
    return status;
  }

  bool fresh(uint64_t heartbeat, int64_t now) const
  {
    const auto found = heartbeats_.find(heartbeat);
    return found != heartbeats_.end() && now >= found->second &&
           now - found->second < heartbeat_age_ns_;
  }

  Arm::Response arm(const Arm::Request & request, int64_t now)
  {
    tick(now);
    Arm::Response response;
    response.generation = status.generation;
    response.message = "Stale session, generation, or heartbeat";
    if (request.actuator_session != status.actuator_session ||
      request.generation != status.generation || request.guard_session.empty() ||
      !fresh(request.heartbeat, now)) return response;

    if (request.operation == Arm::Request::ARM) {
      if (status.armed || status.emergency_stop || status.paused || !status.stationary) {
        response.message = "Actuator must be disarmed, unpaused, stationary, and free of estop";
        return response;
      }
      // Consume this generation: duplicate/delayed arm calls cannot succeed again.
      disarm("armed_waiting_for_new_command");
      status.armed = true;
      status.guard_session = request.guard_session;
      status.last_valid_ns = now;
    } else if (request.operation == Arm::Request::DISARM ||
      request.operation == Arm::Request::ESTOP || request.operation == Arm::Request::CLEAR_ESTOP)
    {
      if (request.operation == Arm::Request::ESTOP) status.emergency_stop = true;
      if (request.operation == Arm::Request::CLEAR_ESTOP) status.emergency_stop = false;
      disarm(request.operation == Arm::Request::ESTOP ? "emergency_stop" : "explicit_disarm");
    } else {
      response.message = "Unknown operation";
      return response;
    }
    response.success = true;
    response.generation = status.generation;
    response.message = status.reason;
    return response;
  }

  bool command(const Command & command, int64_t now)
  {
    tick(now);
    if (!status.armed || command.actuator_session != status.actuator_session ||
      command.generation != status.generation || command.guard_session != status.guard_session ||
      command.sequence <= status.accepted_sequence || !fresh(command.heartbeat, now)) return false;
    const auto & v = command.velocity;
    if (!std::isfinite(v.linear.x) || !std::isfinite(v.angular.z) ||
      std::abs(v.linear.x) > max_linear_ || std::abs(v.angular.z) > max_angular_ ||
      v.linear.y != 0.0 || v.linear.z != 0.0 || v.angular.x != 0.0 || v.angular.y != 0.0)
    {
      disarm("invalid_command");
      return false;
    }
    status.accepted_sequence = command.sequence;
    status.last_valid_ns = now;
    linear = v.linear.x;
    angular = v.angular.z;
    return true;
  }

  Status status;
  double linear{0.0};
  double angular{0.0};

private:
  int64_t timeout_ns_;
  int64_t heartbeat_age_ns_;
  double max_linear_;
  double max_angular_;
  std::map<uint64_t, int64_t> heartbeats_;
};
}  // namespace vehicle_actuator_sim
