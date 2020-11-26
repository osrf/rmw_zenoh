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

#include "rmw_zenoh_cpp/rmw_node_impl.hpp"

void rmw_node_impl_t::topics_and_types_query_handler(zn_query_t * query, void const * arg)
{
    const rmw_node_impl_t * node_data = static_cast<const rmw_node_impl_t *>(arg);
    z_string_t res = zn_query_res_name(query);
    z_string_t pred = zn_query_predicate(query);

    char * result;
    result = (char * )malloc(0);
    size_t total_length = 0;

    for(size_t i = 0; i < node_data->topics_.size; i++)
    {
        size_t current_length = strlen(node_data->topics_.data[i]) + 1;
        result = (char *) realloc(result, total_length + current_length);
        snprintf(result + total_length, current_length, "%s", node_data->topics_.data[i]);
        total_length += current_length;
        result[total_length - 1] = ';';
    }
    result[total_length - 1] = '\0';
    printf(">> [Query handler] Handling '%.*s?%.*s'\n", (int)res.len, res.val, (int)pred.len, pred.val);
    zn_send_reply(query, node_data->uri_, (const uint8_t *)result, total_length);
    free(result);
}
