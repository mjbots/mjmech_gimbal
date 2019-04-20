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

#include "fw/quaternion.h"
#include "fw/static_signal.h"

namespace fw {

struct AhrsData {
  uint32_t timestamp = {};
  int32_t error = 0;
  uint16_t rate_hz = 0;

  Quaternion attitude;
  Euler euler_deg;
  Point3D body_rate_dps;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(error));
    a->Visit(MJ_NVP(rate_hz));
    a->Visit(MJ_NVP(attitude));
    a->Visit(MJ_NVP(euler_deg));
    a->Visit(MJ_NVP(body_rate_dps));
  }
};

typedef StaticSignal<void (const AhrsData*)> AhrsDataSignal;

}
