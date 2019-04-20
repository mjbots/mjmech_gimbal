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

#include "fw/error.h"

namespace fw {

namespace {
struct GimbalErrorCategory : mjlib::micro::error_category {
  const char* name() const noexcept override { return "gimbal"; }
  std::string_view message(int condition) const override {
    switch (static_cast<errc>(condition)) {
      case errc::kSuccess: return "success";
      case errc::kDmaStreamTransferError: return "dma stream transfer error";
      case errc::kDmaStreamFifoError: return "dma stream fifo error";
      case errc::kUartOverrunError: return "uart overrun error";
      case errc::kUartFramingError: return "uart framing error";
      case errc::kUartNoiseError: return "uart noise error";
      case errc::kUartBufferOverrunError: return "uart buffer overrun";
      case errc::kCalibrationFault: return "calibration fault";
      case errc::kMotorDriverFault: return "motor driver fault";
      case errc::kOverVoltage: return "over voltage";
      case errc::kEncoderFault: return "encoder fault";
      case errc::kMotorNotConfigured: return "motor not configured";
      case errc::kPwmCycleOverrun: return "pwm cycle overrun";
      case errc::kOverTemperature: return "over temperature";
    }
    return "unknown";
  }
};

}

const mjlib::micro::error_category& gimbal_error_category() {
  static GimbalErrorCategory result;
  return result;
}

mjlib::micro::error_code make_error_code(errc err) {
  return mjlib::micro::error_code(
      static_cast<int>(err), gimbal_error_category());
}

}
