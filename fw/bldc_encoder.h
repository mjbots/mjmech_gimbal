// Copyright 2016-2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/pool_ptr.h"
#include "mjlib/micro/telemetry_manager.h"

#include "fw/as5048_driver.h"
#include "fw/bldc_encoder_data.h"
#include "fw/millisecond_timer.h"

namespace fw {

/// Interprets a rotary encoder in a way necessary for use as the
/// primary encoder on a BLDC motor.
class BldcEncoder {
 public:
  BldcEncoder(mjlib::micro::Pool&, const std::string_view& name,
              As5048Driver&,
              MillisecondTimer&,
              mjlib::micro::PersistentConfig&,
              mjlib::micro::TelemetryManager&);
  ~BldcEncoder();

  struct Config {
    int8_t sign = 1;
    float offset_deg = 0.0f;

    /// The position in degrees, after applying the above @p
    /// offset_deg, where the phase is 0.0.
    float phase_center_deg = 0.0f;

    /// The number of poles, for the purpose of calculating phase.
    uint8_t poles = 7;

    template <typename Archive>
    void Serialize(Archive* a) {
      a->Visit(MJ_NVP(sign));
      a->Visit(MJ_NVP(offset_deg));
      a->Visit(MJ_NVP(phase_center_deg));
      a->Visit(MJ_NVP(poles));
    }
  };

  void PollMillisecond();

  BldcEncoderDataSignal* data_signal();
  const BldcEncoderData* data() const;

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
