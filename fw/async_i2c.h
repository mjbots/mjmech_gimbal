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

#include "mjlib/base/string_span.h"
#include "mjlib/micro/async_types.h"
#include "mjlib/micro/static_function.h"

namespace fw {

class AsyncI2C {
 public:
  AsyncI2C() {}
  AsyncI2C(const AsyncI2C&) = delete;
  virtual ~AsyncI2C() {}

  virtual void AsyncRead(uint8_t device_address,
                         uint8_t memory_address,
                         mjlib::base::string_span buffer,
                         mjlib::micro::ErrorCallback) = 0;
  virtual void AsyncWrite(uint8_t device_address,
                          uint8_t memory_address,
                          const std::string_view& buffer,
                          mjlib::micro::ErrorCallback) = 0;
};

}
