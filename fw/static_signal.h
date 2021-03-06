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

#include "mjlib/base/assert.h"
#include "mjlib/micro/static_function.h"

namespace fw {

template <typename Signature, std::size_t Size=2>
struct StaticSignal;

template <typename R, typename... Args, std::size_t Size>
struct StaticSignal<R(Args...), Size> {
  StaticSignal() {}

  void Connect(const mjlib::micro::StaticFunction<R(Args...)>& function) {
    for (auto& item: elements_) {
      if (!item.valid()) {
        item = function;
        return;
      }
    }
    MJ_ASSERT(false);
  }

  R operator()(Args... args) const {
    for (auto& item: elements_) {
      if (item.valid()) { item(std::forward<Args>(args)...); }
    }
  }

 private:
  std::array<mjlib::micro::StaticFunction<R(Args...)>, Size> elements_;
};

}
