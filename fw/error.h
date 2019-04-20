// Copyright 2018-2019 Josh Pieper, jjp@pobox.com.
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

#include <type_traits>

#include "mjlib/micro/error_code.h"

namespace fw {

enum class errc {
  kSuccess = 0,

  kDmaStreamTransferError = 1,
  kDmaStreamFifoError = 2,
  kUartOverrunError = 3,
  kUartFramingError = 4,
  kUartNoiseError = 5,
  kUartBufferOverrunError = 6,

  kCalibrationFault = 7,
  kMotorDriverFault = 8,
  kOverVoltage = 9,
  kEncoderFault = 10,
  kMotorNotConfigured = 11,
  kPwmCycleOverrun = 12,
  kOverTemperature = 13,
};

const mjlib::micro::error_category& gimbal_error_category();
mjlib::micro::error_code make_error_code(errc);
}

namespace mjlib {
namespace micro {

template <>
struct is_error_code_enum<fw::errc> : std::true_type {};

}
}
