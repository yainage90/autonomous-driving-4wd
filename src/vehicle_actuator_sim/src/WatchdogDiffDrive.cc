/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "WatchdogDiffDrive.hh"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>

#include <limits>
#include <thread>
#include <random>
#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include "vehicle_actuator_sim/watchdog.hpp"
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/math/DiffDriveOdometry.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/SpeedLimiter.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/JointVelocityCmd.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Velocity command.
struct Commands
{
  /// \brief Linear velocity.
  double lin;

  /// \brief Angular velocity.
  double ang;

  Commands() : lin(0.0), ang(0.0) {}
};

class gz::sim::systems::WatchdogDiffDrivePrivate
{
  /// \brief Update odometry and publish an odometry message.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Update the linear and angular velocities.
  /// \param[in] _info System update information.
  /// \param[in] _ecm The EntityComponentManager of the given simulation
  /// instance.
  public: void UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &_ecm);

  /// \brief Gazebo communication node.
  public: transport::Node node;

  /// \brief Entity of the left joint
  public: std::vector<Entity> leftJoints;

  /// \brief Entity of the right joint
  public: std::vector<Entity> rightJoints;

  /// \brief Name of left joint
  public: std::vector<std::string> leftJointNames;

  /// \brief Name of right joint
  public: std::vector<std::string> rightJointNames;

  /// \brief Calculated speed of left joint
  public: double leftJointSpeed{0};

  /// \brief Calculated speed of right joint
  public: double rightJointSpeed{0};

  /// \brief Distance between wheels
  public: double wheelSeparation{1.0};

  /// \brief Wheel radius
  public: double wheelRadius{0.2};

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief The model's canonical link.
  public: Link canonicalLink{kNullEntity};

  /// \brief Update period calculated from <odom__publish_frequency>.
  public: std::chrono::steady_clock::duration odomPubPeriod{0};

  /// \brief Last sim time odom was published.
  public: std::chrono::steady_clock::duration lastOdomPubTime{0};

  /// \brief Diff drive odometry.
  public: math::DiffDriveOdometry odom;

  /// \brief Diff drive odometry message publisher.
  public: transport::Node::Publisher odomPub;

  /// \brief Diff drive tf message publisher.
  public: transport::Node::Publisher tfPub;

  /// \brief Linear velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterLin;

  /// \brief Angular velocity limiter.
  public: std::unique_ptr<math::SpeedLimiter> limiterAng;

  /// \brief Previous control command.
  public: Commands last0Cmd;

  /// \brief Previous control command to last0Cmd.
  public: Commands last1Cmd;

  public: std::unique_ptr<vehicle_actuator_sim::Watchdog> watchdog;
  public: rclcpp::Context::SharedPtr context;
  public: rclcpp::Node::SharedPtr rosNode;
  public: std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  public: std::thread executorThread;
  public: rclcpp::Subscription<vehicle_interfaces::msg::ActuatorCommand>::SharedPtr commandSub;
  public: rclcpp::Publisher<vehicle_interfaces::msg::ActuatorStatus>::SharedPtr statusPub;
  public: rclcpp::Service<vehicle_interfaces::srv::ArmActuator>::SharedPtr armService;
  public: rclcpp::TimerBase::SharedPtr statusTimer;
  public: int64_t lastUpdate{0};
  public: int64_t stationarySince{0};
  public: bool wasPaused{false};
  public: static int64_t Now() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  public: ~WatchdogDiffDrivePrivate() {
    if (executor) executor->cancel();
    if (executorThread.joinable()) executorThread.join();
    if (context && context->is_valid()) context->shutdown("actuator unloaded");
  }

  /// \brief A mutex to protect the target velocity command.
  public: std::mutex mutex;

  /// \brief frame_id from sdf.
  public: std::string sdfFrameId;

  /// \brief child_frame_id from sdf.
  public: std::string sdfChildFrameId;
};

//////////////////////////////////////////////////
WatchdogDiffDrive::WatchdogDiffDrive()
  : dataPtr(std::make_unique<WatchdogDiffDrivePrivate>())
{
}

//////////////////////////////////////////////////
WatchdogDiffDrive::~WatchdogDiffDrive() = default;

void WatchdogDiffDrive::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &/*_eventMgr*/)
{
  this->dataPtr->model = Model(_entity);

  // Get the canonical link
  std::vector<Entity> links = _ecm.ChildrenByComponents(
      this->dataPtr->model.Entity(), components::CanonicalLink());
  if (!links.empty())
    this->dataPtr->canonicalLink = Link(links[0]);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "DiffDrive plugin should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  auto sdfElem = _sdf->FindElement("left_joint");
  while (sdfElem)
  {
    this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("left_joint");
  }
  sdfElem = _sdf->FindElement("right_joint");
  while (sdfElem)
  {
    this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
    sdfElem = sdfElem->GetNextElement("right_joint");
  }

  this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
      this->dataPtr->wheelSeparation).first;
  this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
      this->dataPtr->wheelRadius).first;

  // Instantiate the speed limiters.
  this->dataPtr->limiterLin = std::make_unique<math::SpeedLimiter>();
  this->dataPtr->limiterAng = std::make_unique<math::SpeedLimiter>();

  // Parse speed limiter parameters.

  // Min Velocity
  if (_sdf->HasElement("min_velocity"))
  {
    const double minVel = _sdf->Get<double>("min_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minVel);
    this->dataPtr->limiterAng->SetMinVelocity(minVel);
  }
  if (_sdf->HasElement("min_linear_velocity"))
  {
    const double minLinVel = _sdf->Get<double>("min_linear_velocity");
    this->dataPtr->limiterLin->SetMinVelocity(minLinVel);
  }
  if (_sdf->HasElement("min_angular_velocity"))
  {
    const double minAngVel = _sdf->Get<double>("min_angular_velocity");
    this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
  }

  // Max Velocity
  if (_sdf->HasElement("max_velocity"))
  {
    const double maxVel = _sdf->Get<double>("max_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
    this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
  }
  if (_sdf->HasElement("max_linear_velocity"))
  {
    const double maxLinVel = _sdf->Get<double>("max_linear_velocity");
    this->dataPtr->limiterLin->SetMaxVelocity(maxLinVel);
  }
  if (_sdf->HasElement("max_angular_velocity"))
  {
    const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
    this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
  }

  // Min Acceleration
  if (_sdf->HasElement("min_acceleration"))
  {
    const double minAccel = _sdf->Get<double>("min_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
    this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
  }
  if (_sdf->HasElement("min_linear_acceleration"))
  {
    const double minLinAccel = _sdf->Get<double>("min_linear_acceleration");
    this->dataPtr->limiterLin->SetMinAcceleration(minLinAccel);
  }
  if (_sdf->HasElement("min_angular_acceleration"))
  {
    const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
    this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
  }

  // Max Acceleration
  if (_sdf->HasElement("max_acceleration"))
  {
    const double maxAccel = _sdf->Get<double>("max_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
  }
  if (_sdf->HasElement("max_linear_acceleration"))
  {
    const double maxLinAccel = _sdf->Get<double>("max_linear_acceleration");
    this->dataPtr->limiterLin->SetMaxAcceleration(maxLinAccel);
  }
  if (_sdf->HasElement("max_angular_acceleration"))
  {
    const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
    this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
  }

  // Min Jerk
  if (_sdf->HasElement("min_jerk"))
  {
    const double minJerk = _sdf->Get<double>("min_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minJerk);
    this->dataPtr->limiterAng->SetMinJerk(minJerk);
  }
  if (_sdf->HasElement("min_linear_jerk"))
  {
    const double minLinJerk = _sdf->Get<double>("min_linear_jerk");
    this->dataPtr->limiterLin->SetMinJerk(minLinJerk);
  }
  if (_sdf->HasElement("min_angular_jerk"))
  {
    const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
    this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
  }

  // Max Jerk
  if (_sdf->HasElement("max_jerk"))
  {
    const double maxJerk = _sdf->Get<double>("max_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
    this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
  }
  if (_sdf->HasElement("max_linear_jerk"))
  {
    const double maxLinJerk = _sdf->Get<double>("max_linear_jerk");
    this->dataPtr->limiterLin->SetMaxJerk(maxLinJerk);
  }
  if (_sdf->HasElement("max_angular_jerk"))
  {
    const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
    this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
  }

  double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
  if (odomFreq > 0)
  {
    std::chrono::duration<double> odomPer{1 / odomFreq};
    this->dataPtr->odomPubPeriod =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
  }

  // Setup odometry.
  this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
      this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);

  // The only velocity input is the session-protected ROS actuator protocol.
  auto & d = *this->dataPtr;
  d.context = std::make_shared<rclcpp::Context>();
  d.context->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(d.context).use_global_arguments(false);
  if (_sdf->HasElement("safety_config_file"))
    options.arguments({"--ros-args", "--params-file", _sdf->Get<std::string>("safety_config_file")});
  d.rosNode = std::make_shared<rclcpp::Node>("actuator_sim", options);
  const auto param = [&d](const char * name, double value) {
    return d.rosNode->declare_parameter<double>(name, value);
  };
  const double period = param("actuator_output_period_sec", 0.05);
  const double timeout = param("actuator_watchdog_sec", 0.3);
  const double age = param("actuator_heartbeat_timeout_sec", 0.2);
  const double statusTimeout = param("actuator_status_timeout_sec", 0.2);
  const double maxLinear = param("max_linear_speed_mps", 0.2);
  const double maxAngular = param("max_angular_speed_rps", 0.6);
  std::random_device random;
  std::ostringstream session;
  for (int i = 0; i < 4; ++i) session << std::hex << random();
  d.watchdog = std::make_unique<vehicle_actuator_sim::Watchdog>(
    session.str(), timeout, age, period, statusTimeout, maxLinear, maxAngular);
  if (!std::isfinite(d.wheelSeparation) || d.wheelSeparation <= 0.0 ||
    !std::isfinite(d.wheelRadius) || d.wheelRadius <= 0.0)
    throw std::invalid_argument("Wheel dimensions must be finite and positive");
  auto qos = rclcpp::QoS(1).reliable().durability_volatile();
  d.statusPub = d.rosNode->create_publisher<vehicle_interfaces::msg::ActuatorStatus>(
    "/actuator/status", qos);
  d.commandSub = d.rosNode->create_subscription<vehicle_interfaces::msg::ActuatorCommand>(
    "/actuator/command", qos, [&d](vehicle_interfaces::msg::ActuatorCommand::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(d.mutex);
      d.watchdog->command(*msg, WatchdogDiffDrivePrivate::Now());
    });
  d.armService = d.rosNode->create_service<vehicle_interfaces::srv::ArmActuator>(
    "/actuator/arm", [&d](const std::shared_ptr<vehicle_interfaces::srv::ArmActuator::Request> req,
      std::shared_ptr<vehicle_interfaces::srv::ArmActuator::Response> res) {
      std::lock_guard<std::mutex> lock(d.mutex);
      const auto now = WatchdogDiffDrivePrivate::Now();
      // Physics must have updated recently; stationary telemetry cannot outlive a stall.
      if (!d.lastUpdate || now - d.lastUpdate >= 200000000)
        d.watchdog->status.stationary = false;
      *res = d.watchdog->arm(*req, now);
    });
  d.statusTimer = d.rosNode->create_wall_timer(std::chrono::duration<double>(period), [&d]() {
    vehicle_interfaces::msg::ActuatorStatus status;
    {
      std::lock_guard<std::mutex> lock(d.mutex);
      status = d.watchdog->heartbeat(WatchdogDiffDrivePrivate::Now());
    }
    d.statusPub->publish(status);
  });
  rclcpp::ExecutorOptions executorOptions;
  executorOptions.context = d.context;
  d.executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executorOptions);
  d.executor->add_node(d.rosNode);
  d.executorThread = std::thread([&d]() { d.executor->spin(); });

  std::vector<std::string> odomTopics;
  if (_sdf->HasElement("odom_topic"))
  {
    odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
  }
  odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
      "/odometry");
  auto odomTopic = validTopic(odomTopics);

  this->dataPtr->odomPub = this->dataPtr->node.Advertise<msgs::Odometry>(
      odomTopic);

  std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
    "/tf"};
  if (_sdf->HasElement("tf_topic"))
    tfTopic = _sdf->Get<std::string>("tf_topic");
  this->dataPtr->tfPub = this->dataPtr->node.Advertise<msgs::Pose_V>(
      tfTopic);

  if (_sdf->HasElement("frame_id"))
    this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");

  if (_sdf->HasElement("child_frame_id"))
    this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");

  gzmsg << "Watchdog actuator receiving /actuator/command" << std::endl;
}

//////////////////////////////////////////////////
void WatchdogDiffDrive::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("WatchdogDiffDrive::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  // If the joints haven't been identified yet, look for them
  static std::set<std::string> warnedModels;
  auto modelName = this->dataPtr->model.Name(_ecm);
  if (this->dataPtr->leftJoints.empty() ||
      this->dataPtr->rightJoints.empty())
  {
    bool warned{false};
    for (const std::string &name : this->dataPtr->leftJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->leftJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find left joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }

    for (const std::string &name : this->dataPtr->rightJointNames)
    {
      Entity joint = this->dataPtr->model.JointByName(_ecm, name);
      if (joint != kNullEntity)
        this->dataPtr->rightJoints.push_back(joint);
      else if (warnedModels.find(modelName) == warnedModels.end())
      {
        gzwarn << "Failed to find right joint [" << name << "] for model ["
                << modelName << "]" << std::endl;
        warned = true;
      }
    }
    if (warned)
    {
      warnedModels.insert(modelName);
    }
  }

  if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
    return;

  if (warnedModels.find(modelName) != warnedModels.end())
  {
    gzmsg << "Found joints for model [" << modelName
           << "], plugin will start working." << std::endl;
    warnedModels.erase(modelName);
  }

  // Expiry and zero-output selection happen before any wheel command is written,
  // including the first update after pause or a stopped simulator process.
  auto & d = *this->dataPtr;
  std::lock_guard<std::mutex> lock(d.mutex);
  const auto now = WatchdogDiffDrivePrivate::Now();
  auto & w = *d.watchdog;
  const auto gap = d.lastUpdate ? now - d.lastUpdate : 0;
  w.status.max_update_gap_ns = std::max(w.status.max_update_gap_ns, gap);
  d.lastUpdate = now;
  if ((_info.paused && !d.wasPaused) || _info.dt < std::chrono::steady_clock::duration::zero())
    w.disarm(_info.paused ? "simulation_paused" : "simulation_rewound");
  d.wasPaused = _info.paused;
  w.status.paused = _info.paused;
  const bool stationary = d.odom.Initialized() &&
    std::abs(d.odom.LinearVelocity()) <= 0.02 && std::abs(*d.odom.AngularVelocity()) <= 0.05;
  if (!stationary || _info.paused || gap >= 200000000) d.stationarySince = 0;
  else if (!d.stationarySince) d.stationarySince = now;
  w.status.stationary = d.stationarySince && now - d.stationarySince >= 1000000000;
  w.tick(now);
  d.UpdateVelocity(_info, _ecm);
  if (!w.status.armed) {
    d.leftJointSpeed = d.rightJointSpeed = 0.0;
    d.last0Cmd = Commands();
    d.last1Cmd = Commands();
    if (w.status.left_output_rps != 0.0 || w.status.right_output_rps != 0.0 ||
      !w.status.zero_applied_ns) w.status.zero_applied_ns = now;
  }
  w.status.left_output_rps = d.leftJointSpeed;
  w.status.right_output_rps = d.rightJointSpeed;


  for (Entity joint : this->dataPtr->leftJoints)
  {
    // skip this entity if it has been removed
    if (!_ecm.HasEntity(joint))
      continue;

    // Update wheel velocity
    _ecm.SetComponentData<components::JointVelocityCmd>(joint,
      {this->dataPtr->leftJointSpeed});
  }

  for (Entity joint : this->dataPtr->rightJoints)
  {
    // skip this entity if it has been removed
    if (!_ecm.HasEntity(joint))
      continue;

    // Update wheel velocity
    _ecm.SetComponentData<components::JointVelocityCmd>(joint,
      {this->dataPtr->rightJointSpeed});
  }

  // Create the left and right side joint position components if they
  // don't exist.
  auto leftPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->leftJoints[0]);
  if (!leftPos && _ecm.HasEntity(this->dataPtr->leftJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->leftJoints[0],
        components::JointPosition());
  }

  auto rightPos = _ecm.Component<components::JointPosition>(
      this->dataPtr->rightJoints[0]);
  if (!rightPos && _ecm.HasEntity(this->dataPtr->rightJoints[0]))
  {
    _ecm.CreateComponent(this->dataPtr->rightJoints[0],
        components::JointPosition());
  }
}

//////////////////////////////////////////////////
void WatchdogDiffDrive::PostUpdate(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("WatchdogDiffDrive::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->mutex);
  this->dataPtr->UpdateOdometry(_info, _ecm);
}

//////////////////////////////////////////////////
void WatchdogDiffDrivePrivate::UpdateOdometry(const UpdateInfo &_info,
    const EntityComponentManager &_ecm)
{
  GZ_PROFILE("WatchdogDiffDrive::UpdateOdometry");
  // Initialize, if not already initialized.
  if (!this->odom.Initialized())
  {
    this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
    return;
  }

  if (this->leftJoints.empty() || this->rightJoints.empty())
    return;

  // Get the first joint positions for the left and right side.
  auto leftPos = _ecm.Component<components::JointPosition>(this->leftJoints[0]);
  auto rightPos = _ecm.Component<components::JointPosition>(
      this->rightJoints[0]);

  // Abort if the joints were not found or just created.
  if (!leftPos || !rightPos || leftPos->Data().empty() ||
      rightPos->Data().empty())
  {
    return;
  }

  this->odom.Update(leftPos->Data()[0], rightPos->Data()[0],
      std::chrono::steady_clock::time_point(_info.simTime));

  // Throttle publishing
  auto diff = _info.simTime - this->lastOdomPubTime;
  if (diff > std::chrono::steady_clock::duration::zero() &&
      diff < this->odomPubPeriod)
  {
    return;
  }
  this->lastOdomPubTime = _info.simTime;

  // Construct the odometry message and publish it.
  msgs::Odometry msg;
  msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
  msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());

  math::Quaterniond orientation(0, 0, *this->odom.Heading());
  msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);

  msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
  msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());

  // Set the time stamp in the header
  msg.mutable_header()->mutable_stamp()->CopyFrom(
      convert<msgs::Time>(_info.simTime));

  // Set the frame id.
  auto frame = msg.mutable_header()->add_data();
  frame->set_key("frame_id");
  if (this->sdfFrameId.empty())
  {
    frame->add_value(this->model.Name(_ecm) + "/odom");
  }
  else
  {
    frame->add_value(this->sdfFrameId);
  }

  std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
  if (this->sdfChildFrameId.empty())
  {
    if (linkName)
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
    }
  }
  else
  {
    auto childFrame = msg.mutable_header()->add_data();
    childFrame->set_key("child_frame_id");
    childFrame->add_value(this->sdfChildFrameId);
  }

  // Construct the Pose_V/tf message and publish it.
  msgs::Pose_V tfMsg;
  msgs::Pose *tfMsgPose = tfMsg.add_pose();
  tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
  tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
  tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());

  // Publish the messages
  this->odomPub.Publish(msg);
  this->tfPub.Publish(tfMsg);
}

//////////////////////////////////////////////////
void WatchdogDiffDrivePrivate::UpdateVelocity(const UpdateInfo &_info,
    const EntityComponentManager &/*_ecm*/)
{
  GZ_PROFILE("WatchdogDiffDrive::UpdateVelocity");

  double linVel = this->watchdog->linear;
  double angVel = this->watchdog->angular;
  if (!this->watchdog->status.armed || _info.paused) {
    this->leftJointSpeed = this->rightJointSpeed = 0.0;
    return;
  }

  // Limit the target velocity if needed.
  this->limiterLin->Limit(
      linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
  this->limiterAng->Limit(
      angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);

  // Update history of commands.
  this->last1Cmd = last0Cmd;
  this->last0Cmd.lin = linVel;
  this->last0Cmd.ang = angVel;

  // Convert the target velocities to joint velocities.
  this->rightJointSpeed =
    (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  this->leftJointSpeed =
    (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
}

GZ_ADD_PLUGIN(WatchdogDiffDrive,
                    System,
                    WatchdogDiffDrive::ISystemConfigure,
                    WatchdogDiffDrive::ISystemPreUpdate,
                    WatchdogDiffDrive::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(WatchdogDiffDrive, "vehicle::WatchdogDiffDrive")
