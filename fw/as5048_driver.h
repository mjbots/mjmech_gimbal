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

#include "mjlib/base/visitor.h"

#include "mjlib/micro/async_types.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/async_i2c.h"
#include "fw/async_spi.h"
#include "fw/millisecond_timer.h"

namespace fw {

class As5048Driver {
 public:
  As5048Driver(mjlib::micro::Pool&, const std::string_view& name,
               AsyncI2C*, AsyncSPI*, MillisecondTimer&,
               mjlib::micro::PersistentConfig&,
               mjlib::micro::TelemetryManager&);
  ~As5048Driver();

  struct Data {
    uint32_t timestamp = 0;
    uint8_t agc = 0;
    uint8_t diagnostics = 0;
    uint16_t magnitude = 0;
    uint16_t angle = 0;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(timestamp));
      a->Visit(MJ_NVP(agc));
      a->Visit(MJ_NVP(diagnostics));
      a->Visit(MJ_NVP(magnitude));
      a->Visit(MJ_NVP(angle));
    }
  };

  void AsyncRead(Data*, mjlib::micro::ErrorCallback);
  void Poll();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
