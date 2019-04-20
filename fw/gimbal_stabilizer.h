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

#include "mjlib/base/pid.h"
#include "mjlib/micro/async_types.h"
#include "mjlib/micro/command_manager.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/ahrs_data.h"
#include "fw/bldc_encoder_data.h"
#include "fw/bldc_pwm.h"
#include "fw/millisecond_timer.h"
#include "fw/gpio_pin.h"

namespace fw {

class GimbalStabilizer {
 public:
  GimbalStabilizer(mjlib::micro::Pool&,
                   MillisecondTimer&,
                   mjlib::micro::PersistentConfig&,
                   mjlib::micro::TelemetryManager&,
                   AhrsDataSignal&,
                   GpioPin& boost_enable,
                   GpioPin& motor_enable,
                   BldcPwm& motor1, BldcPwm& motor2,
                   BldcEncoderDataSignal& yaw_encoder_signal,
                   BldcEncoderDataSignal& pitch_encoder_signal,
                   GpioPin& torque_led);
  ~GimbalStabilizer();

  /// Clear all fault states and start again.
  void Reset();

  /// When set true and not in a fault state, power is applied to the
  /// motors.
  void SetTorque(bool);

  /// Attempt to stabilize the IMU at the given pitch and yaw in the
  /// AHRS reference frame.
  void SetImuAttitude(float pitch_deg, float yaw_deg);

  /// Move the IMU at the following pitch and roll rate, stabilizing
  /// external disturbances.
  void SetImuRate(float pitch_dps, float yaw_dps);

  /// Set an equivalent IMU yaw command so as to position the unit at
  /// the given absolute yaw angle.
  void SetAbsoluteYaw(float yaw_deg);

  /// Re-enter the initialization phase.
  void RestartInitialization();

  void Command(const std::string_view&,
               const mjlib::micro::CommandManager::Response&);

  void PollMillisecond();

  enum State {
    kInitializing,
    kOperating,
    kFault,
    kNumStates,
  };

  static std::array<std::pair<State, const char*>, kNumStates> StateMapper() {
    return { {
        { kInitializing, "kInitializing" },
        { kOperating, "kOperating" },
        { kFault, "kFault" },
      } };
  }

  struct Data {
    State state;
    mjlib::base::PID::State pitch;
    mjlib::base::PID::State yaw;

    uint32_t start_timestamp = 0;

    float unwrapped_yaw_deg = 0.0f;
    float wrap_yaw_offset_deg = 0.0f;

    Euler desired_deg;
    Euler target_deg;
    Point3D desired_body_rate_dps;
    uint32_t last_ahrs_update = 0;
    bool torque_on = false;
    uint32_t last_fault_reason = 0;

    float pitch_command = 0.0f;
    float yaw_command = 0.0f;
    float pitch_power = 0.0f;
    float yaw_power = 0.0f;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_ENUM(state, StateMapper));
      a->Visit(MJ_NVP(pitch));
      a->Visit(MJ_NVP(yaw));
      a->Visit(MJ_NVP(start_timestamp));
      a->Visit(MJ_NVP(unwrapped_yaw_deg));
      a->Visit(MJ_NVP(wrap_yaw_offset_deg));
      a->Visit(MJ_NVP(desired_deg));
      a->Visit(MJ_NVP(target_deg));
      a->Visit(MJ_NVP(desired_body_rate_dps));
      a->Visit(MJ_NVP(last_ahrs_update));
      a->Visit(MJ_NVP(torque_on));
      a->Visit(MJ_NVP(last_fault_reason));
      a->Visit(MJ_NVP(pitch_command));
      a->Visit(MJ_NVP(yaw_command));
      a->Visit(MJ_NVP(pitch_power));
      a->Visit(MJ_NVP(yaw_power));
    }
  };

  const Data& data() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
