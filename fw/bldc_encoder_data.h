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

#include "fw/static_signal.h"

namespace fw {

struct BldcEncoderData {
  uint32_t timestamp = 0;
  float position_deg = 0.0f;
  float phase = 0.0f;

  uint32_t raw_errors = 0;
  uint32_t raw_last_error = 0;
  uint32_t raw_first_error = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(position_deg));
    a->Visit(MJ_NVP(phase));
    a->Visit(MJ_NVP(raw_errors));
    a->Visit(MJ_NVP(raw_last_error));
    a->Visit(MJ_NVP(raw_first_error));
  }
};

typedef StaticSignal<void (const BldcEncoderData*)> BldcEncoderDataSignal;

}
