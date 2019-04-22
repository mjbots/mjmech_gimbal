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

#include "fw/stm32_hal_spi.h"

#include "mjlib/base/assert.h"

#include "fw/error.h"

namespace {
SPI_TypeDef* SelectSPI(int n) {
  if (n == 3) {
    return SPI3;
  }
  MJ_ASSERT(false);
  return nullptr;
}

struct Registry {
  SPI_HandleTypeDef* spi = nullptr;
  fw::Stm32HalSPI::Impl* impl = nullptr;
};

std::array<Registry, 4> g_registry;
}

namespace fw {

class Stm32HalSPI::Impl {
 public:
  Impl(int spi_number, GPIO_TypeDef* cs_gpio, uint16_t cs_pin)
      : cs_gpio_(cs_gpio),
        cs_pin_(cs_pin) {

    hspi_.Instance = SelectSPI(spi_number);

    hspi_.Init.Mode = SPI_MODE_MASTER;
    hspi_.Init.Direction = SPI_DIRECTION_2LINES;
    hspi_.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi_.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi_.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi_.Init.NSS = SPI_NSS_SOFT;
    hspi_.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    hspi_.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi_.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi_.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi_.Init.CRCPolynomial = 10;

    HAL_SPI_Init(&hspi_);

    for (auto& element: g_registry) {
      if (element.spi == nullptr) {
        element.spi = &hspi_;
        element.impl = this;
        return;
      }
    }

    MJ_ASSERT(false);
  }

  ~Impl() {
    for (auto& element: g_registry) {
      if (element.impl == this) {
        element.spi = nullptr;
        element.impl = nullptr;
        return;
      }
    }
    MJ_ASSERT(false);
  }

  void Poll() {
    if (complete_flag_ == kIdle) { return; }

    // Reset the chip select pin.
    HAL_GPIO_WritePin(cs_gpio_, cs_pin_, GPIO_PIN_SET);

    const int code = [&]() {
      switch (complete_flag_) {
        case kCompleted: { return 0; }
        case kError: { return 1; }
        case kIdle: { MJ_ASSERT(false); }
      }
      MJ_ASSERT(false);
      return 0;
    }();

    const auto callback = callback_;
    callback_ = {};
    complete_flag_ = kIdle;
    callback({code, gimbal_error_category()});
  }

  void Complete() { complete_flag_ = kCompleted; }
  void Error() { error_code_ = 1; complete_flag_ = kError; }

  SPI_HandleTypeDef hspi_;
  GPIO_TypeDef* const cs_gpio_;
  const uint16_t cs_pin_;

  mjlib::micro::ErrorCallback callback_;

  enum CompleteFlag {
    kIdle,
    kCompleted,
    kError,
  };

  volatile CompleteFlag complete_flag_ = kIdle;
  volatile int error_code_ = 0;
};

Stm32HalSPI::Stm32HalSPI(mjlib::micro::Pool& pool, int spi_number,
                         GPIO_TypeDef* cs_gpio, uint16_t cs_pin)
: impl_(&pool, spi_number, cs_gpio, cs_pin) {}

Stm32HalSPI::~Stm32HalSPI() {}

void Stm32HalSPI::AsyncTransaction(const std::string_view& tx_buffer,
                                   const mjlib::base::string_span& rx_buffer,
                                   mjlib::micro::ErrorCallback callback) {
  MJ_ASSERT(!impl_->callback_.valid());
  MJ_ASSERT(tx_buffer.size() == static_cast<size_t>(rx_buffer.size()));
  MJ_ASSERT(impl_->complete_flag_ == Impl::kIdle);

  impl_->callback_ = callback;

  // Do chip select.
  HAL_GPIO_WritePin(impl_->cs_gpio_, impl_->cs_pin_, GPIO_PIN_RESET);

  auto result = HAL_SPI_TransmitReceive_DMA(
      &impl_->hspi_,
      reinterpret_cast<uint8_t*>(const_cast<char*>(tx_buffer.data())),
      reinterpret_cast<uint8_t*>(rx_buffer.data()),
      tx_buffer.size());
  if (result != 0) {
    impl_->complete_flag_ = Impl::kError;
    impl_->error_code_ = result;
  }
}

void Stm32HalSPI::Poll() {
  impl_->Poll();
}

extern "C" {
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  for (auto& element: g_registry) {
    if (element.spi == hspi) { element.impl->Complete(); }
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi) {
  for (auto& element: g_registry) {
    if (element.spi == hspi) { element.impl->Error(); }
  }
}
}

}
