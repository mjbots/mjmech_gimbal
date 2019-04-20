// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "pool_ptr.h"

class TelemetryManager;
class Clock;

/// This class keeps track of things like how many main loops we
/// execute per primary event, and other system health issues like
/// memory usage.
class SystemInfo {
 public:
  SystemInfo(Pool&, TelemetryManager&, Clock&);
  ~SystemInfo();

  void MainLoopCount();
  void PollMillisecond();

 private:
  class Impl;
  PoolPtr<Impl> impl_;
};
