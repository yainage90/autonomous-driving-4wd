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
#pragma once
#include <memory>
#include <gz/sim/System.hh>
namespace gz { namespace sim { inline namespace GZ_SIM_VERSION_NAMESPACE { namespace systems {
class WatchdogDiffDrivePrivate;
class WatchdogDiffDrive : public System, public ISystemConfigure,
  public ISystemPreUpdate, public ISystemPostUpdate {
public:
  WatchdogDiffDrive();
  ~WatchdogDiffDrive() override;
  void Configure(const Entity &, const std::shared_ptr<const sdf::Element> &,
    EntityComponentManager &, EventManager &) override;
  void PreUpdate(const UpdateInfo &, EntityComponentManager &) override;
  void PostUpdate(const UpdateInfo &, const EntityComponentManager &) override;
private:
  std::unique_ptr<WatchdogDiffDrivePrivate> dataPtr;
};
}}}}
