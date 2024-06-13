// Copyright 2023 Robert Bosch GmbH and its subsidiaries
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

#include "off_highway_premium_radar_sample/node.hpp"
#include "off_highway_premium_radar_sample/converters/default_converter.hpp"

namespace off_highway_premium_radar_sample
{
typedef Node<off_highway_premium_radar_sample::DefaultConverter> NodeWithDefaultConverter;
}  // namespace off_highway_premium_radar_sample

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(off_highway_premium_radar_sample::NodeWithDefaultConverter)
