#include "rcutils/logging_macros.h"
#include "rcutils/strdup.h"

#include <rmw/validate_full_topic_name.h>
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/error_handling.h"
#include "rmw/event.h"
#include "rmw/rmw.h"

#include "rmw_zenoh_cpp/rmw_context_impl.hpp"
#include "rmw_zenoh_cpp/identifier.hpp"

#include "impl/type_support_common.hpp"
#include "impl/service_impl.hpp"

extern "C"
{

// ANCILLARY FUNCTIONS =========================================================
// STUB
rmw_ret_t
rmw_service_server_is_available(
  const rmw_node_t * node,
  const rmw_client_t * client,
  bool * result)
{
  (void)node;
  (void)client;
  (void)result;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_service_server_is_available");
  return RMW_RET_ERROR;
}

/// CREATE SERVICE SERVER ======================================================
// Create and return an rmw service server
rmw_service_t *
rmw_create_service(
  const rmw_node_t * node,
  const rosidl_service_type_support_t * type_supports,
  const char * service_name,
  const rmw_qos_profile_t * qos_profile)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_create_service");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, nullptr);

  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return nullptr
  );

  RMW_CHECK_ARGUMENT_FOR_NULL(service_name, nullptr);
  if (strlen(service_name) == 0) {
    RMW_SET_ERROR_MSG("service name is empty string");
    return nullptr;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(qos_profile, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // VALIDATE SERVICE NAME =====================================================
  int * validation_result = static_cast<int *>(
    allocator->allocate(sizeof(int), allocator->state)
  );

  rmw_validate_full_topic_name(service_name, validation_result, nullptr);

  if (*validation_result == RMW_TOPIC_VALID
      || qos_profile->avoid_ros_namespace_conventions) {
    allocator->deallocate(validation_result, allocator->state);
  } else {
    RMW_SET_ERROR_MSG("service name is malformed!");
    allocator->deallocate(validation_result, allocator->state);
    return nullptr;
  }

  // OBTAIN TYPESUPPORT ========================================================
  const rosidl_service_type_support_t * type_support = get_service_typesupport_handle(
    type_supports, RMW_ZENOH_CPP_TYPESUPPORT_C
  );

  if (!type_support) {
    type_support = get_service_typesupport_handle(
      type_supports, RMW_ZENOH_CPP_TYPESUPPORT_CPP);
    if (!type_support) {
      RCUTILS_LOG_INFO("%s", service_name);
      RMW_SET_ERROR_MSG("type support not from this implementation");
      return nullptr;
    }
  }

  // CREATE SERVICE ============================================================
  rmw_service_t * service = static_cast<rmw_service_t *>(
    allocator->allocate(sizeof(rmw_service_t), allocator->state)
  );
  if (!service) {
    RMW_SET_ERROR_MSG("failed to allocate rmw_service_t");
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // Populate common members
  service->implementation_identifier = eclipse_zenoh_identifier;  // const char * assignment
  if (!service->implementation_identifier) {
    RMW_SET_ERROR_MSG("failed to allocate implementation identifier");
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service->service_name = rcutils_strdup(service_name, *allocator);
  if (!service->service_name) {
    RMW_SET_ERROR_MSG("failed to allocate service name");
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service->data = static_cast<rmw_service_data_t *>(
    allocator->allocate(sizeof(rmw_service_data_t), allocator->state)
  );
  if (!service->data) {
    RMW_SET_ERROR_MSG("failed to allocate service data");
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // CREATE SERVICE MEMBERS ====================================================
  // Get typed pointer to implementation specific subscription data struct
  auto * service_data = static_cast<rmw_service_data_t *>(service->data);

  // Obtain Zenoh session and create Zenoh resource for response messages
  ZNSession * s = node->context->impl->session;
  service_data->zn_session_ = s;

  size_t zn_response_topic_id = zn_declare_resource(s, service->service_name);
  service_data->zn_response_topic_id_ = zn_response_topic_id;

  // Init type support callbacks
  auto service_members = static_cast<const service_type_support_callbacks_t *>(type_support->data);
  auto request_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->request_members_->data
  );
  auto response_members = static_cast<const message_type_support_callbacks_t *>(
    service_members->response_members_->data
  );

  service_data->typesupport_identifier_ = type_support->typesupport_identifier;
  service_data->request_type_support_impl_ = request_members;
  service_data->response_type_support_impl_ = response_members;

  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "Creating service: %s", service_name);

  // Allocate and in-place assign new typesupport instances
  service_data->request_type_support_ = static_cast<RequestTypeSupport_cpp *>(
    allocator->allocate(sizeof(RequestTypeSupport_cpp *), allocator->state)
  );
  new(service_data->request_type_support_) RequestTypeSupport_cpp(service_members);
  if (!service_data->request_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate RequestTypeSupport");
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  service_data->response_type_support_ = static_cast<ResponseTypeSupport_cpp *>(
    allocator->allocate(sizeof(ResponseTypeSupport_cpp *), allocator->state)
  );
  new(service_data->response_type_support_) ResponseTypeSupport_cpp(service_members);
  if (!service_data->response_type_support_) {
    RMW_SET_ERROR_MSG("failed to allocate ResponseTypeSupport");
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service_data->response_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  // // Assign node pointer
  service_data->node_ = node;

  // Init Zenoh subscriber for request messages
  service_data->zn_request_subscriber_ = zn_declare_subscriber(
    service_data->zn_session_,
    service->service_name,
    zn_subinfo_default(),  // NOTE(CH3): Default for now
    service_data->zn_request_sub_callback
  );

  // Init Zenoh queryable for availability checking
  service_data->zn_queryable_ = zn_declare_queryable(
    s, service->service_name, EVAL, [](ZNQuery * q){}
  );
  if (service_data->zn_queryable_ == 0) {
    RMW_SET_ERROR_MSG("failed to create availability queryable for service");
    allocator->deallocate(service_data->request_type_support_, allocator->state);
    allocator->deallocate(service_data->response_type_support_, allocator->state);
    allocator->deallocate(service->data, allocator->state);
    allocator->deallocate(service, allocator->state);
    return nullptr;
  }

  return service;
}

/// DESTROY SERVICE
// Destroy and deallocate an RMW service server
rmw_ret_t
rmw_destroy_service(rmw_node_t * node, rmw_service_t * service)
{
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_destroy_service");

  // ASSERTIONS ================================================================
  RMW_CHECK_ARGUMENT_FOR_NULL(node, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(service, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    node,
    node->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    service,
    service->implementation_identifier,
    eclipse_zenoh_identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  // OBTAIN ALLOCATOR ==========================================================
  rcutils_allocator_t * allocator = &node->context->options.allocator;

  // CLEANUP ===================================================================
  allocator->deallocate(static_cast<rmw_service_data_t *>(service->data)->request_type_support_,
                        allocator->state);
  allocator->deallocate(static_cast<rmw_service_data_t *>(service->data)->response_type_support_,
                        allocator->state);
  allocator->deallocate(service->data, allocator->state);
  allocator->deallocate(service, allocator->state);

  return RMW_RET_OK;
}

// STUB
rmw_ret_t
rmw_take_request(
  const rmw_service_t * service,
  rmw_service_info_t * request_header,
  void * ros_request,
  bool * taken)
{
  (void)service;
  (void)request_header;
  (void)ros_request;
  (void)taken;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_take_request (STUB)");
  // return RMW_RET_ERROR;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_send_response(
  const rmw_service_t * service,
  rmw_request_id_t * request_header,
  void * ros_response)
{
  (void)service;
  (void)request_header;
  (void)ros_response;
  RCUTILS_LOG_INFO_NAMED("rmw_zenoh_cpp", "rmw_send_response");
  return RMW_RET_ERROR;
}

} // extern "C"