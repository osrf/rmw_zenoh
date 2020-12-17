// Copyright 2020 Open Source Robotics Foundation, Inc.
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

// Doc: http://docs.ros2.org/latest/api/rmw/init_8h.html

#include <cstring>

#include <memory>

#include "rmw/types.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/init.h"
#include "rmw/validate_full_topic_name.h"

#include "rcutils/logging_macros.h"

#include "rmw_zenoh_common_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_common_cpp/rmw_init_options_impl.hpp"

#include "rmw_zenoh_common_cpp/identifier.hpp"

#include "rmw_zenoh_common_cpp/rmw_zenoh_common.hpp"
#include "rmw_zenoh_common_cpp/zenoh-net-interface.h"
#include "rmw_zenoh_common_cpp/MessageTypeSupport.hpp"
#include "rmw_zenoh_common_cpp/debug_helpers.hpp"
#include "rmw_zenoh_common_cpp/pubsub_impl.hpp"
#include "rcutils/strdup.h"
#include "rmw_zenoh_common_cpp/qos.hpp"
#include "rmw_zenoh_common_cpp/type_support_common.hpp"

#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_c/identifier.h"

#include "rmw_zenoh_pico_dynamic_cpp/dynamic_type_support.hpp"

zn_properties_t * configure_connection_mode(rmw_context_t * context)
{
  if (strcmp(context->options.impl->mode, "CLIENT") == 0) {
    return zn_config_client(context->options.impl->session_locator);
  } else {
    RMW_SET_ERROR_MSG("zenoh-pico can only work in client mode");
    return NULL;
  }
}

void configure_session(zn_session_t * session)
{
  // Start the read session session lease loops
  znp_start_read_task(session);
  znp_start_lease_task(session);
}

const char *
rmw_get_implementation_identifier()
{
  return eclipse_zenoh_identifier;
}

const char *
rmw_get_serialization_format()
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_get_serialization_format");
  return nullptr;
}

/// INIT CONTEXT ===============================================================
// Initialize the middleware with the given options, and yielding an context.
//
// rmw_context_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__context__t.html
//
// Starts a new Zenoh session and configures it according to the init options
// with the following environment variables:
//  - RMW_ZENOH_SESSION_LOCATOR: Session TCP locator to use
//  - RMW_ZENOH_MODE: Lets you set the session to be in CLIENT, ROUTER, or PEER mode
//                    (defaults to PEER)
rmw_ret_t
rmw_init(const rmw_init_options_t * options, rmw_context_t * context)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_cpp", "rmw_init");

  // CLEANUP DEFINITIONS =======================================================
  // Store a pointer to the context with a custom deleter that zero inits the
  // context if any initialization steps fail
  std::unique_ptr<rmw_context_t, void (*)(rmw_context_t *)> clean_when_fail(
    context,
    [](rmw_context_t * context) {*context = rmw_get_zero_initialized_context();});

  rmw_ret_t ret = rmw_zenoh_common_init_pre(options, context, eclipse_zenoh_identifier);
  if (ret == RMW_RET_OK) {
    // Create implementation specific context
    rcutils_allocator_t * allocator = &context->options.allocator;

    rmw_context_impl_t * context_impl = static_cast<rmw_context_impl_t *>(
      allocator->allocate(sizeof(rmw_context_impl_t), allocator->state));

    if (!context_impl) {
      RMW_SET_ERROR_MSG("failed to allocate context impl");
      return RMW_RET_BAD_ALLOC;
    }

    // Open configured Zenoh session, then assign it to the context
    zn_properties_t * config = configure_connection_mode(context);
    if (nullptr == config) {
      allocator->deallocate(context_impl, allocator->state);
      return RMW_RET_ERROR;
    }

    zn_session_t * session = zn_open(config);

    if (session == nullptr) {
      RMW_SET_ERROR_MSG("failed to create Zenoh session when starting context");
      allocator->deallocate(context_impl, allocator->state);
      return RMW_RET_ERROR;
    } else {
      context_impl->session = session;
      context_impl->is_shutdown = false;
    }

    // CLEANUP IF PASSED =========================================================
    context->impl = context_impl;
    clean_when_fail.release();

    configure_session(context_impl->session);
  }
  return ret;
}

/// CREATE NODE ================================================================
// Create a node and return a handle to that node.
//
// rmw_node_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__node__t.html
//
// In the case of Zenoh, the only relevant members are name, namespace and implementation
// identifier.
//
// Most likely we will associate a subset of the context session's publishers and subscribers to
// individual nodes, even though to Zenoh it looks like the session is the one holding on to
// all of them.
rmw_node_t *
rmw_create_node(
  rmw_context_t * context,
  const char * name,
  const char * namespace_,
  size_t domain_id,
  bool localhost_only)
{
  return rmw_zenoh_common_create_node(
    context,
    name,
    namespace_,
    domain_id,
    localhost_only,
    eclipse_zenoh_identifier);
}

/// SHUTDOWN CONTEXT ===========================================================
// Shutdown the middleware for a given context.
//
// In this case, closes the associated Zenoh session.
rmw_ret_t
rmw_shutdown(rmw_context_t * context)
{
  return rmw_zenoh_common_shutdown(context, eclipse_zenoh_identifier);
}

/// INIT OPTIONS ===============================================================
// Initialize given init_options with the default values
// and implementation specific values.
//
// rmw_init_options_t Doc: http://docs.ros2.org/latest/api/rmw/structrmw__init__options__t.html
//
// Note: You should call rmw_get_zero_initialized_init_options()
// to get a zero initialized rmw_init_options_t struct first
rmw_ret_t
rmw_init_options_init(rmw_init_options_t * init_options, rcutils_allocator_t allocator)
{
  return rmw_zenoh_common_init_options_init(init_options, allocator, eclipse_zenoh_identifier);
}

/// FINALIZE CONTEXT ===========================================================
// Finalize a context. (Cleanup and deallocation.)
rmw_ret_t
rmw_context_fini(rmw_context_t * context)
{
  return rmw_zenoh_common_context_fini(context, eclipse_zenoh_identifier);
}

/// DESTROY NODE ===============================================================
// Finalize a given node handle, reclaim the resources, and deallocate the node handle.
rmw_ret_t
rmw_destroy_node(rmw_node_t * node)
{
  return rmw_zenoh_common_destroy_node(node, eclipse_zenoh_identifier);
}

/// COPY OPTIONS ===============================================================
// Copy the given source init options to the destination init options.
rmw_ret_t
rmw_init_options_copy(const rmw_init_options_t * src, rmw_init_options_t * dst)
{
  return rmw_zenoh_common_init_options_copy(src, dst, eclipse_zenoh_identifier);
}

/// FINALIZE OPTIONS ===========================================================
// Finalize the given init_options. (Cleanup and deallocation.)
rmw_ret_t
rmw_init_options_fini(rmw_init_options_t * init_options)
{
  return rmw_zenoh_common_init_options_fini(init_options, eclipse_zenoh_identifier);
}

/// CREATE SUBSCRIPTION ========================================================
// Create and return an rmw subscriber
rmw_subscription_t *
rmw_create_subscription(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_subscription_options_t * subscription_options)
{
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_common_cpp",
    "[rmw_create_subscription] %s with queue of depth %ld",
    topic_name,
    qos_profile->depth);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (strlen(topic_name) == 0) {
    RMW_SET_ERROR_MSG("subscription topic is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  // NOTE(CH3): For some reason the tests want a failed publisher init on passing an unknown QoS.
  // I don't understand why, and I don't see a check to fulfill that test in any RMW implementations
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_create_subscriber() qos_profile:");
  rmw_zenoh_common_cpp::log_debug_qos_profile(qos_profile);

  RMW_CHECK_ARGUMENT_FOR_NULL(subscription_options, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // Although we do not yet support QoS we still fail on clearly-bad settings
  if (!rmw_zenoh_common_cpp::is_valid_qos(qos_profile)) {
    return nullptr;
  }

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE TOPIC NAME =======================================================
  int validation_result;

  if (rmw_validate_full_topic_name(topic_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }

  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("subscription topic is malformed: %s", topic_name);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C);

  if (!type_support) {
    type_support = get_message_typesupport_handle(type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      RCUTILS_LOG_INFO("%s", topic_name);
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  // CREATE SUBSCRIPTION =======================================================
  rmw_subscription_t * subscription = static_cast<rmw_subscription_t *>(
    allocator->allocate(sizeof(rmw_subscription_t), allocator->state));
  if (!subscription) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_subscription_t");
    return nullptr;
  }

  // Populate common members
  subscription->implementation_identifier = eclipse_zenoh_identifier;  // const char * assignment
  subscription->options = *subscription_options;
  subscription->can_loan_messages = false;

  subscription->topic_name = rcutils_strdup(topic_name, *allocator);
  if (!subscription->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate subscription topic name");
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  subscription->data = static_cast<rmw_subscription_data_t *>(
    allocator->allocate(sizeof(rmw_subscription_data_t), allocator->state));
  new(subscription->data) rmw_subscription_data_t();
  if (!subscription->data) {
    RMW_SET_ERROR_MSG("failed to allocate subscription data");
    allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // CREATE SUBSCRIPTION MEMBERS ===============================================
  // Init type support callbacks
  auto * callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Obtain Zenoh session
  zn_session_t * session = node->context->impl->session;

  // Get typed pointer to implementation specific subscription data struct
  auto * subscription_data = static_cast<rmw_subscription_data_t *>(subscription->data);

  subscription_data->zn_session_ = session;
  subscription_data->typesupport_identifier_ = type_support->typesupport_identifier;
  subscription_data->type_support_impl_ = type_support->data;

  // Allocate and in-place assign new message typesupport instance
  subscription_data->type_support_ = static_cast<rmw_zenoh_common_cpp::MessageTypeSupport *>(
    allocator->allocate(sizeof(rmw_zenoh_common_cpp::MessageTypeSupport), allocator->state));
  new(subscription_data->type_support_) rmw_zenoh_common_cpp::MessageTypeSupport(callbacks);
  if (!subscription_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    allocator->deallocate(subscription->data, allocator->state);

    allocator->deallocate(const_cast<char *>(subscription->topic_name), allocator->state);
    allocator->deallocate(subscription, allocator->state);
    return nullptr;
  }

  // Assign node pointer
  subscription_data->node_ = node;

  // Assign and increment unique subscription ID atomically
  subscription_data->subscription_id_ =
    rmw_subscription_data_t::subscription_id_counter.fetch_add(1, std::memory_order_relaxed);

  // Configure message queue
  subscription_data->queue_depth_ = qos_profile->depth;

  // ADD SUBSCRIPTION DATA TO TOPIC MAP ========================================
  // This will allow us to access the subscription data structs for this Zenoh topic key expression
  std::string key(subscription->topic_name);
  auto map_iter = rmw_subscription_data_t::zn_topic_to_sub_data.find(key);

  if (map_iter == rmw_subscription_data_t::zn_topic_to_sub_data.end()) {
    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_common_cpp",
      "[rmw_create_subscription] New topic detected: %s",
      topic_name);

    // If no elements for this Zenoh topic key expression exists, add it in
    std::vector<rmw_subscription_data_t *> sub_data_vec{subscription_data};
    rmw_subscription_data_t::zn_topic_to_sub_data[key] = sub_data_vec;

    // We initialise subscribers ONCE (otherwise we'll get duplicate messages)
    // The topic name will be the same for any duplicate subscribers, so it is ok
    subscription_data->zn_subscriber_ = zn_declare_subscriber(
      subscription_data->zn_session_,
      zn_rname(subscription->topic_name),
      zn_subinfo_default(),  // NOTE(CH3): Default for now
      subscription_data->zn_sub_callback,
      nullptr);

    RCUTILS_LOG_DEBUG_NAMED(
      "rmw_zenoh_common_cpp",
      "[rmw_create_subscription] Zenoh subscription declared for %s",
      topic_name);
  } else {
    // Otherwise, append to the vector
    map_iter->second.push_back(subscription_data);
  }

  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_common_cpp",
    "[rmw_create_subscription] Subscription for %s (ID: %ld) added to topic map",
    topic_name,
    subscription_data->subscription_id_);

  // TODO(CH3): Put the subscription name/pointer into its corresponding node for tracking?

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return subscription;
}

/// DESTROY SUBSCRIPTION =======================================================
// Destroy and deallocate an RMW subscription
rmw_ret_t
rmw_destroy_subscription(
  rmw_node_t * node,
  rmw_subscription_t * subscription)
{
  return rmw_zenoh_common_destroy_subscription(
    node,
    subscription,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_subscription_get_actual_qos(
  const rmw_subscription_t * subscription,
  rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_subscription_get_actual_qos(
    subscription,
    qos_profile,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_subscription_count_matched_publishers(
  const rmw_subscription_t * subscription,
  size_t * count)
{
  return rmw_zenoh_common_subscription_count_matched_publishers(
    subscription,
    count,
    eclipse_zenoh_identifier);
}

/// CREATE PUBLISHER ===========================================================
// Create and return an rmw publisher.
rmw_publisher_t *
rmw_create_publisher(
  const rmw_node_t * node,
  const rosidl_message_type_support_t * type_supports,
  const char * topic_name,
  const rmw_qos_profile_t * qos_profile,
  const rmw_publisher_options_t * publisher_options)
{
  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "[rmw_create_publisher] %s", topic_name);

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (strlen(topic_name) == 0) {
    RMW_SET_ERROR_MSG("publisher topic is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);

  // TODO(CH3): When we figure out how to spoof QoS, check for a 'configured' QoS to pass the final
  // test that is failing
  //
  // if (# SOME CHECK HERE) {
  //   RMW_SET_ERROR_MSG("expected configured QoS profile");
  //   return nullptr;
  // }

  RCUTILS_LOG_DEBUG_NAMED("rmw_zenoh_common_cpp", "rmw_create_publisher() qos_profile:");
  rmw_zenoh_common_cpp::log_debug_qos_profile(qos_profile);

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // Although we do not yet support QoS we still fail on clearly-bad settings
  if (!rmw_zenoh_common_cpp::is_valid_qos(qos_profile)) {
    return nullptr;
  }

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE TOPIC NAME =======================================================
  int validation_result;

  if (rmw_validate_full_topic_name(topic_name, &validation_result, nullptr) != RMW_RET_OK) {
    RMW_SET_ERROR_MSG("rmw_validate_full_topic_name failed");
    return nullptr;
  }

  if (validation_result != RMW_TOPIC_VALID && !qos_profile->avoid_ros_namespace_conventions) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("publisher topic is malformed: %s", topic_name);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_message_type_support_t * type_support = get_message_typesupport_handle(
    type_supports, rosidl_typesupport_introspection_c__identifier);

  if (!type_support) {
    type_support = get_message_typesupport_handle(
      type_supports, rosidl_typesupport_introspection_cpp::typesupport_identifier);
    if (!type_support) {
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  // CREATE PUBLISHER ==========================================================
  rmw_publisher_t * publisher = static_cast<rmw_publisher_t *>(
    allocator->allocate(sizeof(rmw_publisher_t), allocator->state));
  if (!publisher) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_publisher_t");
    return nullptr;
  }

  // Populate common members
  publisher->implementation_identifier = type_support->typesupport_identifier;

  publisher->topic_name = rcutils_strdup(topic_name, *allocator);
  if (!publisher->topic_name) {
    RMW_SET_ERROR_MSG("failed to allocate publisher topic name");
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  publisher->data = static_cast<rmw_publisher_data_t *>(
    allocator->allocate(sizeof(rmw_publisher_data_t), allocator->state));
  if (!publisher->data) {
    RMW_SET_ERROR_MSG("failed to allocate publisher data");
    allocator->deallocate(publisher->data, allocator->state);
    allocator->deallocate(const_cast<char *>(publisher->topic_name), allocator->state);
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  publisher->options = *publisher_options;

  // CREATE PUBLISHER MEMBERS ==================================================
  // Get typed pointer to implementation specific publisher data struct
  auto publisher_data = static_cast<rmw_publisher_data_t *>(publisher->data);

  // Init type support callbacks
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);

  // Create Zenoh resource
  zn_session_t * session = node->context->impl->session;

  // The topic ID must be unique within a single process, but separate processes can reuse IDs,
  // even in the same Zenoh network, because the ID is never transmitted over the wire.
  // Conversely, the ID used in two communicating processes cannot be used to determine if they are
  // using the same topic or not.
  publisher_data->zn_topic_id_ = zn_declare_resource(session, zn_rname(publisher->topic_name));

  // Assign publisher data members
  publisher_data->zn_session_ = session;
  publisher_data->typesupport_identifier_ = type_support->typesupport_identifier;
  publisher_data->type_support_impl_ = type_support->data;
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_zenoh_common_cpp",
    "[rmw_create_publisher] Zenoh resource declared: %s (%ld)",
    topic_name,
    publisher_data->zn_topic_id_);

  // Assign node pointer
  publisher_data->node_ = node;

  // Allocate and in-place construct new message typesupport instance
  std::string type_name = _create_type_name(
    type_support->data, publisher_data->typesupport_identifier_);
  if (!_get_registered_type(publisher_data->node_, type_name, &publisher_data->type_support_)) {
    publisher_data->type_support_ = _create_message_type_support(type_support->data,
        publisher_data->typesupport_identifier_);
    _register_type(publisher_data->node_, publisher_data->type_support_, publisher_data->typesupport_identifier_);
  }

  if (!publisher_data->type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate MessageTypeSupport");
    allocator->deallocate(publisher->data, allocator->state);

    allocator->deallocate(const_cast<char *>(publisher->topic_name), allocator->state);
    allocator->deallocate(publisher, allocator->state);
    return nullptr;
  }

  // TODO(CH3): Put the publisher name/pointer into its corresponding node for tracking?

  // NOTE(CH3) TODO(CH3): No graph updates are implemented yet
  // I am not sure how this will work with Zenoh
  //
  // Perhaps track something using the nodes?

  return publisher;
}

/// DESTROY PUBLISHER ==========================================================
// Destroy and deallocate an rmw publisher.
rmw_ret_t
rmw_destroy_publisher(
  rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  return rmw_zenoh_common_destroy_publisher(
    node,
    publisher,
    eclipse_zenoh_identifier);
}

rmw_ret_t
rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos_profile)
{
  return rmw_zenoh_common_publisher_get_actual_qos(
    publisher,
    qos_profile,
    eclipse_zenoh_identifier);
}
