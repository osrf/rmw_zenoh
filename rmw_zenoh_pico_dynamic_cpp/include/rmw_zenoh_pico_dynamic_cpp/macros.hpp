// Copyright 2018 Intel Corporation All rights reserved.
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

#ifndef RMW_DPS_CPP__MACROS_HPP_
#define RMW_DPS_CPP__MACROS_HPP_

#include <limits>
#include <string>

#define SPECIALIZE_GENERIC_C_SEQUENCE(C_NAME, C_TYPE) \
  template<> \
  struct GenericCSequence<C_TYPE> \
  { \
    using type = rosidl_runtime_c__ ## C_NAME ## __Sequence; \
 \
    static void fini(type * sequence) { \
      rosidl_runtime_c__ ## C_NAME ## __Sequence__fini(sequence); \
    } \
 \
    static bool init(type * sequence, size_t size) { \
      return rosidl_runtime_c__ ## C_NAME ## __Sequence__init(sequence, size); \
    } \
  };

#endif  // RMW_DPS_CPP__MACROS_HPP_
