// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_ZENOH_COMMON_CPP__PARTICIPANT_HPP_
#define RMW_ZENOH_COMMON_CPP__PARTICIPANT_HPP_

#include "rmw/types.h"

#include "rmw_zenoh_common_cpp/custom_session_info.hpp"
#include "rmw_zenoh_common_cpp/visibility_control.h"

namespace rmw_zenoh_common_cpp
{

RMW_ZENOH_COMMON_CPP_PUBLIC
CustomSessionInfo *
create_session(
  const char * identifier);

RMW_ZENOH_COMMON_CPP_PUBLIC
rmw_ret_t
destroy_session(CustomSessionInfo * info);

}  // namespace rmw_zenoh_common_cpp

#endif  // RMW_ZENOH_COMMON_CPP__PARTICIPANT_HPP_
