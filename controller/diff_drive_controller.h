// Copyright 2024 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2024-12-30
//  Author: daohu527

#pragma once

#include <memory>

#include "common/leadlag_controller.h"
#include "common/trajectory_analyzer.h"
#include "controller/controller.h"
#include "controller/pid_controller.h"

namespace apollo {
namespace control {

class DiffDriveController : public Controller {
 public:
  DiffDriveController();
  virtual ~DiffDriveController();

  common::Status Init(std::shared_ptr<DependencyInjector> injector,
                      const ControlConf *control_conf) override;

  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      control::ControlCommand *cmd) override;

  common::Status Reset() override;

  std::string Name() override;

  void Stop() override;

 private:
  void CloseLogFile();
  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory,
                                 const double preview_time, const double ts,
                                 SimpleLongitudinalDebug *debug);
  virtual bool Shifting();
  virtual bool EPB();

 private:
  const localization::LocalizationEstimate *localization_ = nullptr;
  const canbus::Chassis *chassis_ = nullptr;
  const planning::ADCTrajectory *trajectory_message_ = nullptr;

  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;

  PIDController station_pid_controller_;

  std::shared_ptr<DependencyInjector> injector_;

  std::string name_;
};

}  // namespace control
}  // namespace apollo
