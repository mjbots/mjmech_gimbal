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

#include "fw/stm32_analog_sampler.h"

#include "stm32f4xx_hal.h"

namespace fw {

namespace {
// The resistor divider for the external voltages is 10k by 100k, or a
// factor of 11.
int kExternalDivider = 11;
}

class Stm32AnalogSampler::Impl {
 public:
  Impl(MillisecondTimer& clock,
       mjlib::micro::PersistentConfig& persistent_config,
       mjlib::micro::TelemetryManager& telemetry)
      : clock_(clock) {
    ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of c \
       onversion)
    */
    hadc_.Instance = ADC1;
    hadc_.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
    hadc_.Init.Resolution = ADC_RESOLUTION12b;
    hadc_.Init.ScanConvMode = DISABLE;
    hadc_.Init.ContinuousConvMode = DISABLE;
    hadc_.Init.DiscontinuousConvMode = DISABLE;
    hadc_.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc_.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc_.Init.NbrOfConversion = 1;
    hadc_.Init.DMAContinuousRequests = DISABLE;
    hadc_.Init.EOCSelection = EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc_);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and \
       its sample time.
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc_, &sConfig);

    data_updater_ = telemetry.Register("power", &data_);
  }

  void PollMillisecond() {
    if (configuring_) {
      HAL_ADC_Start(&hadc_);
      configuring_ = false;
      return;
    }
    if (state_ != kIdle) {
      // We are looking for a conversion to have completed.
      if (!__HAL_ADC_GET_FLAG(&hadc_, ADC_FLAG_EOC)) { return; }

      const uint16_t value = HAL_ADC_GetValue(&hadc_);
      switch (state_) {
        case kIdle: { MJ_ASSERT(false); break; }
        case kVRefInt: { data_.raw_vrefint = value; break; }
        case kVBat: { data_.raw_vbat = value; break; }
        case k8V: { data_.raw_8v = value; break; }
        case k12V: { data_.raw_12v = value; break; }
        case kTemperature: { data_.raw_temperature = value; break; }
      }
    }

    if (state_ == kFinal) {
      // Emit our result.
      data_.timestamp = clock_.read_us();
      const float vadc = 1.2f / (data_.raw_vrefint / 4096.0f);
      data_.power_vbat = data_.raw_vbat / 4096.0f * 4 * vadc;
      data_.power_8v = data_.raw_8v / 4096.0f * kExternalDivider * vadc;
      data_.power_12v = data_.raw_12v / 4096.0f * kExternalDivider * vadc;

      const float vtemp = data_.raw_temperature / 4096.0 * vadc;
      data_.temperature_C = (vtemp - 0.75) / 0.00275 + 25;

      data_updater_();
    }

    // Finally, advance the state and start our next conversion.
    const int next_state = std::max(1, (state_ + 1) % (kFinal + 1));
    state_ = static_cast<State>(next_state);

    __attribute__((unused)) ADC_ChannelConfTypeDef adc_conf;
    adc_conf.Rank = 1;
    adc_conf.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    adc_conf.Channel = [&]() -> uint32_t {
      switch (state_) {
        case kIdle: { MJ_ASSERT(false); return 0; }
        case kVRefInt: { return ADC_CHANNEL_VREFINT; }
        case kVBat: { return ADC_CHANNEL_VBAT; }
        case k8V: { return ADC_CHANNEL_11; }
        case k12V: { return ADC_CHANNEL_12; }
        case kTemperature: { return ADC_CHANNEL_TEMPSENSOR; }
      }
      MJ_ASSERT(false);
      return 0;
    }();

    // The stupid HAL layer sets, but never clears these flags, so we
    // can't switch between things without clearing them ourselves.
    ADC->CCR &= ~ADC_CCR_TSVREFE;
    ADC->CCR &= ~ADC_CCR_VBATE;
    HAL_ADC_ConfigChannel(&hadc_, &adc_conf);
    configuring_ = true;
  }

  MillisecondTimer& clock_;
  Data data_;
  mjlib::micro::StaticFunction<void ()> data_updater_;
  ADC_HandleTypeDef hadc_;

  enum State {
    kIdle,
    kVRefInt,
    kVBat,
    k8V,
    k12V,
    kTemperature,
    kFinal = kTemperature,
  };

  State state_ = kIdle;
  bool configuring_ = false;
};

Stm32AnalogSampler::Stm32AnalogSampler(
    mjlib::micro::Pool& pool,
    MillisecondTimer& clock,
    mjlib::micro::PersistentConfig& persistent_config,
    mjlib::micro::TelemetryManager& telemetry)
: impl_(&pool, clock, persistent_config, telemetry) {
}

Stm32AnalogSampler::~Stm32AnalogSampler() {}

void Stm32AnalogSampler::PollMillisecond() {
  impl_->PollMillisecond();
}

}
