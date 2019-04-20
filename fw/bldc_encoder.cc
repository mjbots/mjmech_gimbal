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

#include "fw/bldc_encoder.h"

#include "fw/math_util.h"

namespace fw {

class BldcEncoder::Impl {
 public:
  Impl(const std::string_view& name,
       As5048Driver& as5048,
       MillisecondTimer& clock,
       mjlib::micro::PersistentConfig& config,
       mjlib::micro::TelemetryManager& telemetry)
      : as5048_(as5048),
        clock_(clock) {
    config.Register(name, &config_, [](){});
    data_updater_ = telemetry.Register(name, &data_);
  }

  void PollMillisecond() {
    if (read_outstanding_) { return; }
    read_outstanding_ = true;
    as5048_.AsyncRead(&raw_data_, [this](auto ec) {
        this->HandleRawRead(ec);
      });
  }

  void HandleRawRead(mjlib::micro::error_code ec) {
    read_outstanding_ = false;

    data_.timestamp = clock_.read_us();

    if (ec) {
      data_.raw_errors++;
      data_.raw_last_error = ec.value();
      if (data_.raw_first_error == 0) {
        data_.raw_first_error = ec.value();
      }

      data_updater_();
      return;
    }

    data_.timestamp = clock_.read_us();
    const float unwrapped_deg =
        config_.sign * 360.0f * raw_data_.angle / 16384.0f +
        config_.offset_deg;
    data_.position_deg = Degrees(WrapNegPiToPi(Radians(unwrapped_deg)));

    const float unwrapped_phase =
        (data_.position_deg - config_.phase_center_deg) /
        (360.0f / config_.poles);
    data_.phase =
        ((unwrapped_phase < 0.0) ? 1.0 : 0.0) +
        std::fmod(unwrapped_phase, 1.0);

    data_updater_();
    data_signal_(&data_);
  }

  As5048Driver& as5048_;
  MillisecondTimer& clock_;
  Config config_;

  BldcEncoderDataSignal data_signal_;
  BldcEncoderData data_;
  mjlib::micro::StaticFunction<void ()> data_updater_;
  bool read_outstanding_ = false;
  As5048Driver::Data raw_data_;
};

BldcEncoder::BldcEncoder(
    mjlib::micro::Pool& pool,
    const std::string_view& name,
    As5048Driver& as5048,
    MillisecondTimer& clock,
    mjlib::micro::PersistentConfig& config,
    mjlib::micro::TelemetryManager& telemetry)
    : impl_(&pool, name, as5048, clock, config, telemetry) {}

BldcEncoder::~BldcEncoder() {}

void BldcEncoder::PollMillisecond() {
  impl_->PollMillisecond();
}

BldcEncoderDataSignal* BldcEncoder::data_signal() {
  return &impl_->data_signal_;
}

const BldcEncoderData* BldcEncoder::data() const {
  return &impl_->data_;
}

}
