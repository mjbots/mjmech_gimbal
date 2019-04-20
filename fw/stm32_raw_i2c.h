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

#include "mjlib/micro/pool_ptr.h"

#include "fw/async_i2c.h"
#include "fw/gpio_pin.h"
#include "fw/millisecond_timer.h"

namespace fw {

class Stm32RawI2C : public AsyncI2C {
 public:
  struct Parameters;

  Stm32RawI2C(mjlib::micro::Pool&, int i2c_number, GpioPin& scl, GpioPin& sda,
              const Parameters&, MillisecondTimer&);
  virtual ~Stm32RawI2C();

  void AsyncRead(uint8_t device_address,
                 uint8_t memory_address,
                 mjlib::base::string_span,
                 mjlib::micro::ErrorCallback) override;
  void AsyncWrite(uint8_t device_address,
                  uint8_t memory_address,
                  const std::string_view&,
                  mjlib::micro::ErrorCallback) override;

  void Poll();

  struct Parameters {
    int speed = 400000;
    enum DutyCycle {
      kDuty2,
      kDuty16_9,
    };
    DutyCycle duty_cycle = kDuty2;
  };

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
