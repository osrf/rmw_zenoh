// Copyright 2020 Continental AG
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

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "internal/common.hpp"
#include "internal/serialization/deserializer.hpp"

namespace zenoh
{
  namespace rmw
  {

    class CppDeserializer : public Deserializer
    {
      const rosidl_typesupport_introspection_cpp::MessageMembers* members_;

      template <typename T>
      void DeserializeSingle(char* member, const char** serialized_data);

      array_size_t DeserializeArraySize(const char** serialized_data);

      template <typename T>
      void DeserializeArray(char* member, size_t size, const char** serialized_data);

      template <typename T>
      void DeserializeArray(char* message,
                            const rosidl_typesupport_introspection_cpp::MessageMember* member,
                            const char** serialized_data);

      template <typename T>
      void DeserializeDynamicArray(char* member, const char** serialized_data);

      template <typename T>
      void DeserializeDynamicArray(char* message,
                                   const rosidl_typesupport_introspection_cpp::MessageMember* member,
                                   const char** serialized_data);

      template <typename T>
      void Deserialize(char* member_data,
                       const rosidl_typesupport_introspection_cpp::MessageMember* member,
                       const char** serialized_data);

      void DeserializeMessage(const char** serialized_data,
                              const rosidl_typesupport_introspection_cpp::MessageMembers* members,
                              char* message);

    public:
      explicit CppDeserializer(const rosidl_typesupport_introspection_cpp::MessageMembers* members)
        : members_(members)
      {
      }

      virtual void Deserialize(void* message, const void* serialized_data, size_t size) override;
    };

  } // namespace rmw
} // namespace zenoh
