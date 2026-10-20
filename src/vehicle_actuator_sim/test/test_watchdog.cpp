#include <gtest/gtest.h>
#include <limits>
#include "vehicle_actuator_sim/watchdog.hpp"

using vehicle_actuator_sim::Watchdog;
using Arm = Watchdog::Arm;

class Receiver : public ::testing::Test {
protected:
  Watchdog w{"device", 0.3, 0.2, 0.05, 0.2, 0.2, 0.6};
  int64_t now{1000000000};
  Watchdog::Command command;
  Arm::Request request(uint8_t operation) {
    const auto status = w.heartbeat(now);
    Arm::Request req;
    req.actuator_session = status.actuator_session;
    req.generation = status.generation;
    req.guard_session = "guard";
    req.heartbeat = status.heartbeat;
    req.operation = operation;
    return req;
  }
  void activate() {
    w.status.stationary = true;
    EXPECT_TRUE(w.arm(request(Arm::Request::ARM), now).success);
    const auto status = w.heartbeat(now);
    command.actuator_session = status.actuator_session;
    command.generation = status.generation;
    command.guard_session = "guard";
    command.heartbeat = status.heartbeat;
    command.sequence = 1;
    command.velocity.linear.x = 0.1;
  }
};

TEST_F(Receiver, StartupAndStationaryInterlock) {
  EXPECT_FALSE(w.status.armed);
  EXPECT_FALSE(w.arm(request(Arm::Request::ARM), now).success);
  EXPECT_DOUBLE_EQ(w.linear, 0.0);
}

TEST_F(Receiver, ExpiryBoundaryAndNoRestartOnTraffic) {
  activate();
  ASSERT_TRUE(w.command(command, now));
  w.tick(now + 299999999);
  EXPECT_TRUE(w.status.armed);
  w.tick(now + 300000000);
  EXPECT_FALSE(w.status.armed);
  EXPECT_EQ(w.status.expired_ns, now + 300000000);
  EXPECT_DOUBLE_EQ(w.linear, 0.0);
  ++command.sequence;
  command.heartbeat = w.heartbeat(now + 300000001).heartbeat;
  EXPECT_FALSE(w.command(command, now + 300000001));
}

TEST_F(Receiver, DuplicateAndReorderedMessagesDoNotRenewWatchdog) {
  activate();
  command.sequence = 10;
  ASSERT_TRUE(w.command(command, now));
  EXPECT_FALSE(w.command(command, now + 100000000));
  --command.sequence;
  EXPECT_FALSE(w.command(command, now + 150000000));
  EXPECT_EQ(w.status.last_valid_ns, now);
}

TEST_F(Receiver, DistinctDelayedMessagesDoNotRenewWatchdog) {
  activate();
  ASSERT_TRUE(w.command(command, now));
  ++command.sequence;
  EXPECT_FALSE(w.command(command, now + 200000000));
  EXPECT_EQ(w.status.last_valid_ns, now);
}

TEST_F(Receiver, InvalidVelocityDisarmsAndCannotRearmWithOldRequest) {
  auto old = request(Arm::Request::ARM);
  activate();
  command.velocity.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(w.command(command, now));
  EXPECT_FALSE(w.status.armed);
  EXPECT_EQ(w.status.reason, "invalid_command");
  EXPECT_FALSE(w.arm(old, now).success);
}

TEST_F(Receiver, OldSessionAndGenerationCannotMoveOrExtendDeadline) {
  activate();
  auto old = command;
  w.disarm("test");
  activate();
  EXPECT_FALSE(w.command(old, now));
  command.actuator_session = "old_device";
  EXPECT_FALSE(w.command(command, now));
}

TEST_F(Receiver, EstopRequiresSeparateClearAndNewActivation) {
  activate();
  EXPECT_TRUE(w.arm(request(Arm::Request::ESTOP), now).success);
  EXPECT_FALSE(w.arm(request(Arm::Request::ARM), now).success);
  EXPECT_TRUE(w.status.emergency_stop);
  EXPECT_TRUE(w.arm(request(Arm::Request::CLEAR_ESTOP), now).success);
  EXPECT_FALSE(w.status.armed);
  EXPECT_TRUE(w.arm(request(Arm::Request::ARM), now).success);
  EXPECT_DOUBLE_EQ(w.linear, 0.0);
}

TEST_F(Receiver, PauseRejectsActivation) {
  w.status.stationary = true;
  w.status.paused = true;
  EXPECT_FALSE(w.arm(request(Arm::Request::ARM), now).success);
}

TEST(Settings, InvalidTimingRejected) {
  EXPECT_THROW(Watchdog("s", 0.3, 0.4, 0.05, 0.2, 0.2, 0.6), std::invalid_argument);
  EXPECT_THROW(Watchdog("s", 0.3, 0.2, 0.05, 0.2, -0.2, 0.6), std::invalid_argument);
  EXPECT_THROW(Watchdog("s", 0.3, 0.2, 0.05, 0.2,
    std::numeric_limits<double>::infinity(), 0.6), std::invalid_argument);
}
