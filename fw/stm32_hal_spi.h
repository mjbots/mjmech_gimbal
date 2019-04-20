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

#include "mjlib/micro/pool_ptr.h"

#include "fw/async_spi.h"

namespace fw {

class Stm32HalSPI : public AsyncSPI {
 public:
  Stm32HalSPI(mjlib::micro::Pool&, int spi_number,
              GPIO_TypeDef* cs_gpio, uint16_t cs_pin);
  virtual ~Stm32HalSPI();

  void AsyncTransaction(const std::string_view& tx_buffer,
                        const mjlib::base::string_span& rx_buffer,
                        mjlib::micro::ErrorCallback) override;

  void Poll();

  class Impl;
 private:
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
