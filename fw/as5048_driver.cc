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

#include "mjlib/micro/error_code.h"

#include "fw/as5048_driver.h"

namespace fw {

namespace {
const auto u8 = [](char c) { return static_cast<uint8_t>(c); };

struct Config {
  uint8_t i2c_address = 0x80;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(MJ_NVP(i2c_address));
  }
};

enum AS5048 {
  REG_AGC = 0xfa,
  REG_DIAG = 0xfb,
  REG_MAG = 0xfc,
  REG_ANGLE = 0xfe,

  SPI_NOP = 0x0000,
  SPI_AGC = 0x3ffd,
  SPI_MAG = 0x3ffe,
  SPI_ANGLE = 0x3fff,
};

void MakeSpiRead(char* buffer, uint16_t addr) {
  MJ_ASSERT(addr <= 0x3fff);

  uint8_t* const out = reinterpret_cast<uint8_t*>(buffer);
  const uint8_t lsb = addr & 0xff;
  const uint8_t msb = ((addr >> 8) & 0xff) | 0x40;

  uint8_t parity = lsb ^ msb;
  parity = parity ^ (parity >> 4);
  parity = parity ^ (parity >> 2);
  parity = parity ^ (parity >> 1);

  // Ensure even parity.
  out[0] = msb | ((parity & 0x01) ? 0x80 : 0x00);
  out[1] = lsb;
}
}

class As5048Driver::Impl {
 public:
  Impl(const std::string_view& name,
       AsyncI2C* async_i2c,
       AsyncSPI* async_spi,
       MillisecondTimer& clock,
       mjlib::micro::PersistentConfig& config,
       mjlib::micro::TelemetryManager& telemetry)
      : async_i2c_(async_i2c),
        async_spi_(async_spi),
        clock_(clock) {
    config.Register(name, &config_, [](){});
    data_updater_ = telemetry.Register(name, &data_);
    const bool i2c = async_i2c_ != nullptr;
    const bool spi = async_spi_ != nullptr;
    // We require exactly one of an I2C or a SPI interface.
    MJ_ASSERT(i2c ^ spi);

    MakeSpiRead(&spi_tx_buffer_[0], SPI_AGC);
    MakeSpiRead(&spi_tx_buffer_[2], SPI_MAG);
    MakeSpiRead(&spi_tx_buffer_[4], SPI_ANGLE);
    MakeSpiRead(&spi_tx_buffer_[6], SPI_NOP);
  }

  void AsyncRead(Data* data, mjlib::micro::ErrorCallback callback) {
    user_data_ = data;
    user_callback_ = callback;
    if (async_i2c_) {
      ReadI2C();
    } else if (async_spi_) {
      ReadSPI();
    } else {
      MJ_ASSERT(false);
    }
  }

  void ReadI2C() {
    async_i2c_->AsyncRead(config_.i2c_address,
                          REG_AGC,
                          mjlib::base::string_span(buffer_, 6),
                          [this](mjlib::micro::error_code error) {
                            this->HandleReadI2C(error);
                          });
  }

  void HandleReadI2C(mjlib::micro::error_code error) {
    if (error) {
      UserCallback(error);
      return;
    }

    HandleBuffer(&buffer_[0]);
  }

  void HandleBuffer(const char* buf) {
    data_.timestamp = clock_.read_us();
    data_.agc = u8(buf[0]);
    data_.diagnostics = u8(buf[1]);
    data_.magnitude = u8(buf[2]) << 6 | (u8(buf[3]) & 0x3f);
    data_.angle = u8(buf[4]) << 6 | (u8(buf[5]) & 0x3f);

    Emit();
  }

  void HandleSPIBuffer(const char* buf) {
    data_.timestamp = clock_.read_us();
    data_.diagnostics = u8(buf[0] & 0x0f);
    data_.agc = u8(buf[1]);
    data_.magnitude = ((u8(buf[2]) & 0x3f) << 8) | u8(buf[3]);
    data_.angle = ((u8(buf[4]) & 0x3f) << 8) | u8(buf[5]);

    Emit();
  }

  void ReadSPI() {
    MJ_ASSERT(!spi_in_progress_);
    MJ_ASSERT(spi_read_position_ == -1);
    spi_read_position_ = 0;
  }

  void HandleReadSPI(mjlib::micro::error_code error) {
    spi_in_progress_ = false;
    if (error) {
      UserCallback(error);
      return;
    }

    if (spi_read_position_ == 8) {
      spi_read_position_ = -1;
      HandleSPIBuffer(&buffer_[2]);
      std::memset(buffer_, 0, sizeof(buffer_));
    } else {
      // The next poll will start another read.
    }
  }

  void Emit() {
    data_updater_();
    UserCallback({});
  }

  void UserCallback(mjlib::micro::error_code error) {
    auto callback = user_callback_;
    user_callback_ = {};
    if (!error) {
      *user_data_ = data_;
    }
    user_data_ = nullptr;
    callback(error);
  }

  void Poll() {
    if (spi_read_position_ < 0) { return; }
    if (spi_in_progress_) { return; }

    const int8_t this_read = spi_read_position_;
    spi_read_position_ += 2;
    spi_in_progress_ = true;
    async_spi_->AsyncTransaction(
        std::string_view(spi_tx_buffer_ + this_read, 2),
        mjlib::base::string_span(buffer_ + this_read, 2),
        [this](mjlib::micro::error_code error) { this->HandleReadSPI(error); });
  }

  AsyncI2C* const async_i2c_;
  AsyncSPI* const async_spi_;

  MillisecondTimer& clock_;

  Config config_;

  Data data_;
  mjlib::micro::StaticFunction<void ()> data_updater_;

  Data* user_data_ = nullptr;
  mjlib::micro::ErrorCallback user_callback_;

  char spi_tx_buffer_[8] = {};
  int8_t spi_read_position_ = -1;
  bool spi_in_progress_ = false;
  char buffer_[8] = {};
};

As5048Driver::As5048Driver(
    mjlib::micro::Pool& pool,
    const std::string_view& name,
    AsyncI2C* async_i2c, AsyncSPI* async_spi,
    MillisecondTimer& clock,
    mjlib::micro::PersistentConfig& config,
    mjlib::micro::TelemetryManager& telemetry)
: impl_(&pool, name, async_i2c, async_spi, clock, config, telemetry) {
}

As5048Driver::~As5048Driver() {}

void As5048Driver::AsyncRead(Data* data, mjlib::micro::ErrorCallback callback) {
  impl_->AsyncRead(data, callback);
}

void As5048Driver::Poll() {
  impl_->Poll();
}

}
