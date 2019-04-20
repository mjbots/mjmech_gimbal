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

#include "stm32f4xx_hal.h"

#include "fw/async_i2c.h"

namespace fw {

class Stm32HalI2C : public AsyncI2C {
 public:
  Stm32HalI2C(I2C_HandleTypeDef*);
  virtual ~Stm32HalI2C();

  void AsyncRead(uint8_t device_address,
                 uint8_t memory_address,
                 mjlib::base::string_span buffer,
                 mjlib::micro::ErrorCallback) override;
  void AsyncWrite(uint8_t device_address,
                  uint8_t memory_address,
                  const std::string_view& buffer,
                  mjlib::micro::ErrorCallback) override;
  void Poll();

  void ReceiveComplete();
  void TransmitComplete();
  void Error();

 private:
  I2C_HandleTypeDef* const hi2c_;
  bool tx_complete_ = false;
  bool rx_complete_ = false;
  mjlib::micro::ErrorCallback read_callback_;
  mjlib::micro::ErrorCallback write_callback_;
};

}
