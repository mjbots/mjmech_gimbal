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

#include "fw/system_info.h"

#include "mjlib/base/visitor.h"
#include "mjlib/micro/static_function.h"

namespace fw {

namespace {
struct SystemInfoData {
  uint32_t timestamp = 0;
  uint32_t main_loops_per_10ms = 0;
  uint32_t pool_size = 0;
  uint32_t pool_available = 0;
  uint32_t heap_size = 0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(timestamp));
    a->Visit(MJ_NVP(main_loops_per_10ms));
    a->Visit(MJ_NVP(pool_size));
    a->Visit(MJ_NVP(pool_available));
    a->Visit(MJ_NVP(heap_size));
  }
};
}

class SystemInfo::Impl {
 public:
  Impl(mjlib::micro::Pool& pool,
       mjlib::micro::TelemetryManager& telemetry,
       MillisecondTimer& clock)
      : pool_(pool),
        clock_(clock) {
    data_updater_ = telemetry.Register("system_info", &data_);
  }

  void PollMillsecond() {
    ms_count_++;
    if (ms_count_ >= 10) {
      ms_count_ = 0;
    } else {
      return;
    }

    data_.main_loops_per_10ms = main_loops_;
    main_loops_ = 0;

    data_.timestamp = clock_.read_us();
    data_.pool_size = pool_.size();
    data_.pool_available = pool_.available();

    // extern char _heap_start;
    // extern char* heap_end;

    // data_.heap_size = heap_end - &_heap_start;

    data_updater_();
  }

  mjlib::micro::Pool& pool_;
  MillisecondTimer& clock_;

  uint8_t ms_count_ = 0;
  uint32_t main_loops_ = 0;
  SystemInfoData data_;
  mjlib::micro::StaticFunction<void ()> data_updater_;
};

SystemInfo::SystemInfo(mjlib::micro::Pool& pool,
                       mjlib::micro::TelemetryManager& telemetry,
                       MillisecondTimer& clock)
    : impl_(&pool, pool, telemetry, clock) {}

SystemInfo::~SystemInfo() {}

void SystemInfo::MainLoopCount() {
  impl_->main_loops_++;
}

void SystemInfo::PollMillisecond() {
  impl_->PollMillsecond();
}

}
