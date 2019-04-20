// Copyright 2015-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#pragma once

#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/millisecond_timer.h"
#include "fw/gpio_pin.h"
#include "fw/pwm_pin.h"

namespace fw {

class FireControl {
 public:
  FireControl(mjlib::micro::Pool&, MillisecondTimer&,
              mjlib::micro::PersistentConfig&,
              mjlib::micro::TelemetryManager&,
              GpioPin& laser_enable,
              GpioPin& pwm_enable,
              PwmPin& aeg_pwm,
              PwmPin& agitator_pwm,
              GpioPin& arm_switch,
              GpioPin& arm_led);
  ~FireControl();

  void SetLaser(bool);
  void SetFire(uint8_t pwm, uint8_t time_100ms);
  void SetAgitator(uint8_t pwm);

  bool laser() const;
  uint8_t fire_pwm() const;
  uint8_t fire_time_100ms() const;
  uint8_t agitator_pwm() const;

  void Command(const std::string_view&,
               const mjlib::micro::CommandManager::Response&);

  void PollMillisecond();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
