// Copyright 2020 ADLINK, Inc.
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

#include "rmw_zenoh_dynamic_cpp/get_subscriber.hpp"

#include "rmw_zenoh_common_cpp/custom_subscriber_info.hpp"
#include "rmw_zenoh_dynamic_cpp/identifier.hpp"

namespace rmw_zenoh_dynamic_cpp
{

zn_subscriber_t *
get_subscriber(rmw_subscription_t * subscription)
{
  if (!subscription) {
    return nullptr;
  }
  if (subscription->implementation_identifier != eclipse_zenoh_identifier) {
    return nullptr;
  }
  auto impl = static_cast<CustomSubscriberInfo *>(subscription->data);
  return impl->subscriber_;
}

}  // namespace rmw_zenoh_dynamic_cpp
