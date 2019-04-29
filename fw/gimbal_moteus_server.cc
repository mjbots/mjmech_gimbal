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

#include "fw/gimbal_moteus_server.h"

namespace fw {
class GimbalMoteusServer::Impl {
 public:
};

GimbalMoteusServer::GimbalMoteusServer(
    mjlib::micro::Pool& pool,
    GimbalStabilizer&,
    MahonyImu&,
    BldcEncoder&,
    FireControl&)
    : impl_(&pool) {}

GimbalMoteusServer::~GimbalMoteusServer() {}

uint32_t GimbalMoteusServer::Write(Format::Register, const Format::Value&) {
  return {};
}

GimbalMoteusServer::Format::ReadResult GimbalMoteusServer::Read(
    Format::Register, size_t) const {
  return {};
}

void GimbalMoteusServer::Poll() {
}

}
