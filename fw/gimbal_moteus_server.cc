// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "fw/gimbal_moteus_server.h"

#include "mjlib/base/limit.h"

using mjlib::base::Limit;

namespace fw {
namespace {

enum class Register {
  kMode = 0x000,

  kImuPitch = 0x010,
  kImuYaw = 0x011,
  kAbsolutePitch = 0x12,
  kAbsoluteYaw = 0x13,
  kPitchRate = 0x14,
  kYawRate = 0x15,

  kPitchCommand = 0x020,
  kYawCommand = 0x021,
  kAbsolutePitchCommand = 0x022,
  kAbsoluteYawCommand = 0x023,

  kPitchRateCommand = 0x030,
  kYawRateCommand = 0x031,

  kFireTime = 0x040,
  kFirePwm = 0x041,
  kAgitatorPwm = 0x042,
  kLaser = 0x043,
  kBiasCommand = 0x044,
};

using Value = GimbalMoteusServer::Format::Value;

template <typename T>
Value IntMapping(T value, size_t type) {
  switch (type) {
    case 0: return static_cast<int8_t>(value);
    case 1: return static_cast<int16_t>(value);
    case 2: return static_cast<int32_t>(value);
    case 3: return static_cast<float>(value);
  }
  MJ_ASSERT(false);
  return static_cast<int8_t>(0);
}

template <typename T>
Value ScaleSaturate(float value, float scale) {
  if (!std::isfinite(value)) {
    return std::numeric_limits<T>::min();
  }

  const float scaled = value / scale;
  const auto max = std::numeric_limits<T>::max();
  // We purposefully limit to +- max, rather than to min.  The minimum
  // value for our two's complement types is reserved for NaN.
  return Limit<T>(static_cast<T>(scaled), -max, max);
}

Value ScaleMapping(float value,
                   float int8_scale, float int16_scale, float int32_scale,
                   size_t type) {
  switch (type) {
    case 0: return ScaleSaturate<int8_t>(value, int8_scale);
    case 1: return ScaleSaturate<int16_t>(value, int16_scale);
    case 2: return ScaleSaturate<int32_t>(value, int32_scale);
    case 3: return Value(value);
  }
  MJ_ASSERT(false);
  return Value(static_cast<int8_t>(0));
}

Value AngleMapping(float value, size_t type) {
  return ScaleMapping(value, 0.2f, 0.01f, 0.001f, type);
}

Value AngularRateMapping(float value, size_t type) {
  return ScaleMapping(value, 1.0f, 0.01f, 0.001f, type);
}

int8_t ReadIntMapping(Value value) {
  return std::visit([](auto a) {
      return static_cast<int8_t>(a);
    }, value);
}

struct ValueScaler {
  float int8_scale;
  float int16_scale;
  float int32_scale;

  float operator()(int8_t value) const {
    if (value == std::numeric_limits<int8_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int8_scale;
  }

  float operator()(int16_t value) const {
    if (value == std::numeric_limits<int16_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int16_scale;
  }

  float operator()(int32_t value) const {
    if (value == std::numeric_limits<int32_t>::min()) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return value * int32_scale;
  }

  float operator()(float value) const {
    return value;
  }
};

float ReadAngle(Value value) {
  return std::visit(ValueScaler{0.2f, 0.01f, 0.001f}, value);
}

float ReadAngularRate(Value value) {
  return std::visit(ValueScaler{1.0f, 0.01f, 0.001f}, value);
}
}

class GimbalMoteusServer::Impl {
 public:
  Impl(GimbalStabilizer& stabilizer,
       MahonyImu& imu,
       BldcEncoder& yaw_encoder,
       FireControl& fire_control)
      : stabilizer_(stabilizer),
        imu_(imu),
        yaw_encoder_(yaw_encoder),
        fire_control_(fire_control) {}

  uint32_t Write(Format::Register reg, const Format::Value& value) {
    switch (static_cast<Register>(reg)) {
      case Register::kMode: {
        // We'll map 0 to torque off, and anything else to torque on.
        const auto new_mode = ReadIntMapping(value);

        stabilizer_.SetTorque(new_mode != 0);

        return 0;
      }
      case Register::kImuPitch:
      case Register::kImuYaw:
      case Register::kAbsolutePitch:
      case Register::kAbsoluteYaw:
      case Register::kPitchRate:
      case Register::kYawRate: {
        // These are not writeable.
        return 2;
      }

      case Register::kPitchCommand: {
        stabilizer_.SetImuAttitude(
            ReadAngle(value), stabilizer_.data().desired_deg.yaw);
        return 0;
      }
      case Register::kYawCommand: {
        stabilizer_.SetImuAttitude(
            stabilizer_.data().desired_deg.pitch, ReadAngle(value));
        return 0;
      }
      case Register::kAbsolutePitchCommand: {
        return 3;
      }
      case Register::kAbsoluteYawCommand: {
        stabilizer_.SetAbsoluteYaw(ReadAngle(value));
        return 0;
      }
      case Register::kPitchRateCommand: {
        stabilizer_.SetImuRate(
            ReadAngularRate(value), -stabilizer_.data().desired_body_rate_dps.z);
        return 0;
      }
      case Register::kYawRateCommand: {
        stabilizer_.SetImuRate(
            stabilizer_.data().desired_body_rate_dps.x, ReadAngularRate(value));
        return 0;
      }
      case Register::kFireTime: {
        fire_time_ = ReadIntMapping(value);
        return 0;
      }
      case Register::kFirePwm: {
        fire_control_.SetFire(ReadIntMapping(value), fire_time_);
        return 0;
      }
      case Register::kAgitatorPwm: {
        fire_control_.SetAgitator(ReadIntMapping(value));
        return 0;
      }
      case Register::kLaser: {
        fire_control_.SetLaser(ReadIntMapping(value) != 0);
        return 0;
      }
      case Register::kBiasCommand: {
        if (ReadIntMapping(value) != 0) {
          imu_.RestartBiasInitialization();
        }
        return 0;
      }
    }

    // If we got here, then we had an unknown register.
    return 1;
  }

  Format::ReadResult Read(Format::Register reg, size_t type_index) const {
    switch (static_cast<Register>(reg)) {
      case Register::kMode: {
        return IntMapping(stabilizer_.data().torque_on ? 1 : 0, type_index);
      }
      case Register::kImuPitch: {
        return AngleMapping(imu_.data().euler_deg.pitch, type_index);
      }
      case Register::kImuYaw: {
        return AngleMapping(imu_.data().euler_deg.yaw, type_index);
      }
      case Register::kAbsolutePitch: {
        return IntMapping(0, type_index);
      }
      case Register::kAbsoluteYaw: {
        return AngleMapping(yaw_encoder_.data()->position_deg, type_index);
      }
      case Register::kPitchRate: {
        return AngularRateMapping(imu_.data().body_rate_dps.x, type_index);
      }
      case Register::kYawRate: {
        return AngularRateMapping(-imu_.data().body_rate_dps.z, type_index);
      }
      case Register::kPitchCommand: {
        return AngleMapping(stabilizer_.data().desired_deg.pitch, type_index);
      }
      case Register::kYawCommand: {
        return AngleMapping(stabilizer_.data().desired_deg.yaw, type_index);
      }
      case Register::kAbsolutePitchCommand: {
        return AngleMapping(0.0f, type_index);
      }
      case Register::kAbsoluteYawCommand: {
        return AngleMapping(0.0f, type_index);
      }
      case Register::kPitchRateCommand: {
        return AngularRateMapping(stabilizer_.data().desired_body_rate_dps.x,
                                  type_index);
      }
      case Register::kYawRateCommand: {
        return AngularRateMapping(-stabilizer_.data().desired_body_rate_dps.z,
                                  type_index);
      }

      case Register::kFireTime: {
        return IntMapping(fire_control_.fire_time_100ms(), type_index);
      }
      case Register::kFirePwm: {
        return IntMapping(fire_control_.fire_pwm(), type_index);
      }
      case Register::kAgitatorPwm: {
        return IntMapping(fire_control_.agitator_pwm(), type_index);
      }
      case Register::kLaser: {
        return IntMapping(fire_control_.laser() ? 1 : 0, type_index);
      }
      case Register::kBiasCommand: {
        return IntMapping(0, type_index);
      }
    }

    // Unknown register.
    return static_cast<uint32_t>(2);
  }

  void Poll() {
  }

  GimbalStabilizer& stabilizer_;
  MahonyImu& imu_;
  BldcEncoder& yaw_encoder_;
  FireControl& fire_control_;

  uint8_t fire_time_ = 0;
};

GimbalMoteusServer::GimbalMoteusServer(
    mjlib::micro::Pool& pool,
    GimbalStabilizer& stabilizer,
    MahonyImu& imu,
    BldcEncoder& yaw_encoder,
    FireControl& fire_control)
    : impl_(&pool, stabilizer, imu, yaw_encoder, fire_control) {}

GimbalMoteusServer::~GimbalMoteusServer() {}

uint32_t GimbalMoteusServer::Write(
    Format::Register reg,
    const Format::Value& value) {
  return impl_->Write(reg, value);
}

GimbalMoteusServer::Format::ReadResult GimbalMoteusServer::Read(
    Format::Register reg, size_t type_index) const {
  return impl_->Read(reg, type_index);
}

void GimbalMoteusServer::Poll() {
  impl_->Poll();
}

}
