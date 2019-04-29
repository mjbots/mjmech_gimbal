// Copyright 2019 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mjlib/multiplex/micro_server.h"

#include "fw/gimbal_stabilizer.h"
#include "fw/mahony_imu.h"
#include "fw/bldc_encoder.h"
#include "fw/fire_control.h"

namespace fw {

class GimbalMoteusServer : public mjlib::multiplex::MicroServer::Server {
 public:
  using Format = mjlib::multiplex::Format;

  GimbalMoteusServer(mjlib::micro::Pool&,
                     GimbalStabilizer&,
                     MahonyImu&,
                     BldcEncoder& yaw_encoder,
                     FireControl&);
  ~GimbalMoteusServer() override;

  uint32_t Write(Format::Register, const Format::Value&) override;
  Format::ReadResult Read(Format::Register, size_t type_index) const override;

  void Poll();

 private:
  class Impl;
  mjlib::micro::PoolPtr<Impl> impl_;
};

}
