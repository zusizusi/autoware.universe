// Copyright 2020 Tier IV, Inc.
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

#include "manager.hpp"

#include <autoware/behavior_velocity_planner_common/utilization/boost_geometry_helper.hpp>
#include <autoware/behavior_velocity_planner_common/utilization/util.hpp>
#include <autoware_lanelet2_extension/utility/utilities.hpp>
#include <autoware_utils/ros/parameter.hpp>

#include <lanelet2_core/primitives/BasicRegulatoryElements.h>

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace autoware::behavior_velocity_planner
{
using autoware_utils::get_or_declare_parameter;

RoundaboutModuleManager::RoundaboutModuleManager(rclcpp::Node & node)
: SceneModuleManagerInterfaceWithRTC(
    node, getModuleName(),
    getEnableRTC(node, std::string(getModuleName()) + ".enable_rtc.roundabout"))
{
  const std::string ns(RoundaboutModuleManager::getModuleName());
  auto & rp = roundabout_param_;

  // common
  {
    rp.common.attention_area_length =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_length");
    rp.common.attention_area_margin =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_margin");
    rp.common.attention_area_angle_threshold =
      get_or_declare_parameter<double>(node, ns + ".common.attention_area_angle_threshold");
    rp.common.default_stopline_margin =
      get_or_declare_parameter<double>(node, ns + ".common.default_stopline_margin");
    rp.common.path_interpolation_ds =
      get_or_declare_parameter<double>(node, ns + ".common.path_interpolation_ds");
    rp.common.max_accel = get_or_declare_parameter<double>(node, ns + ".common.max_accel");
    rp.common.max_jerk = get_or_declare_parameter<double>(node, ns + ".common.max_jerk");
    rp.common.delay_response_time =
      get_or_declare_parameter<double>(node, ns + ".common.delay_response_time");
    rp.common.enable_pass_judge_before_default_stopline = get_or_declare_parameter<bool>(
      node, ns + ".common.enable_pass_judge_before_default_stopline");
  }

  // collision_detection
  {
    rp.collision_detection.consider_wrong_direction_vehicle = get_or_declare_parameter<bool>(
      node, ns + ".collision_detection.consider_wrong_direction_vehicle");
    rp.collision_detection.collision_detection_hold_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_detection_hold_time");
    rp.collision_detection.min_predicted_path_confidence = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.min_predicted_path_confidence");
    rp.collision_detection.collision_start_margin_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_start_margin_time");
    rp.collision_detection.collision_end_margin_time = get_or_declare_parameter<double>(
      node, ns + ".collision_detection.collision_end_margin_time");

    // target_type
    {
      rp.collision_detection.target_type.car =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.car");
      rp.collision_detection.target_type.bus =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bus");
      rp.collision_detection.target_type.truck =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.truck");
      rp.collision_detection.target_type.trailer =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.trailer");
      rp.collision_detection.target_type.motorcycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.motorcycle");
      rp.collision_detection.target_type.bicycle =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.bicycle");
      rp.collision_detection.target_type.unknown =
        get_or_declare_parameter<bool>(node, ns + ".collision_detection.target_type.unknown");
    }

    // velocity_profile
    {
      rp.collision_detection.velocity_profile.use_upstream = get_or_declare_parameter<bool>(
        node, ns + ".collision_detection.velocity_profile.use_upstream");
      rp.collision_detection.velocity_profile.minimum_upstream_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_upstream_velocity");
      rp.collision_detection.velocity_profile.default_velocity = get_or_declare_parameter<double>(
        node, ns + ".collision_detection.velocity_profile.default_velocity");
      rp.collision_detection.velocity_profile.minimum_default_velocity =
        get_or_declare_parameter<double>(
          node, ns + ".collision_detection.velocity_profile.minimum_default_velocity");
    }

    rp.collision_detection.avoid_collision_by_acceleration.object_time_margin_to_collision_point =
      get_or_declare_parameter<double>(
        node, ns +
                ".collision_detection.avoid_collision_by_acceleration.object_time_margin_to_"
                "collision_point");
  }

  rp.debug.ttc = get_or_declare_parameter<std::vector<int64_t>>(node, ns + ".debug.ttc");

  decision_state_pub_ =
    node.create_publisher<std_msgs::msg::String>("~/debug/roundabout/decision_state", 1);
}

void RoundaboutModuleManager::launchNewModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto routing_graph = planner_data_->route_handler_->getRoutingGraphPtr();
  const auto lanelet_map = planner_data_->route_handler_->getLaneletMapPtr();

  const auto lanelets =
    planning_utils::getLaneletsOnPath(path, lanelet_map, planner_data_->current_odometry->pose);

  for (size_t i = 0; i < lanelets.size(); i++) {
    const auto ll = lanelets.at(i);
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    // Is roundabout?
    const std::string roundabout_attr = ll.attributeOr("roundabout", "else");
    const auto is_roundabout = roundabout_attr == "entry";
    if (!is_roundabout) {
      continue;
    }

    if (hasSameParentLaneletAndTurnDirectionWithRegistered(ll)) {
      continue;
    }
    const auto associative_ids =
      getAssociativeRoundaboutEntryLanelets(ll, lanelet_map, routing_graph);

    const auto new_module = std::make_shared<RoundaboutModule>(
      module_id, lane_id, planner_data_, roundabout_param_, associative_ids, node_,
      logger_.get_child("roundabout_module"), clock_, time_keeper_, planning_factor_interface_);
    generate_uuid(module_id);
    /* set RTC status as non_occluded status initially */
    const UUID uuid = getUUID(new_module->getModuleId());
    rtc_interface_.updateCooperateStatus(
      uuid, true, State::WAITING_FOR_EXECUTION, std::numeric_limits<double>::lowest(),
      std::numeric_limits<double>::lowest(), clock_->now());
    registerModule(std::move(new_module));
  }
}

std::function<bool(const std::shared_ptr<SceneModuleInterfaceWithRTC> &)>
RoundaboutModuleManager::getModuleExpiredFunction(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto lane_set = planning_utils::getLaneletsOnPath(
    path, planner_data_->route_handler_->getLaneletMapPtr(), planner_data_->current_odometry->pose);

  return [lane_set](const std::shared_ptr<SceneModuleInterfaceWithRTC> & scene_module) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    const auto & associative_ids = roundabout_module->getAssociativeIds();
    for (const auto & lane : lane_set) {
      const std::string roundabout_attr = lane.attributeOr("roundabout", "else");
      const auto is_roundabout =
        roundabout_attr == "entry" || roundabout_attr == "internal" || roundabout_attr == "exit";
      if (!is_roundabout) {
        continue;
      }

      if (associative_ids.find(lane.id()) != associative_ids.end() /* contains */) {
        return false;
      }
    }
    return true;
  };
}

// TODO(iwasawa): Refactor this function to use regulatory elements
std::set<lanelet::Id> RoundaboutModuleManager::getAssociativeRoundaboutEntryLanelets(
  const lanelet::ConstLanelet & lane, const lanelet::LaneletMapPtr lanelet_map,
  const lanelet::routing::RoutingGraphPtr routing_graph)
{
  const std::string roundabout_attr = lane.attributeOr("roundabout", "else");
  if (roundabout_attr != "entry") {
    return {};
  }

  const auto parents = routing_graph->previous(lane);
  std::set<lanelet::Id> parent_neighbors;
  for (const auto & parent : parents) {
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_velocity_planner"),
      "Parent lanelet found: %ld", parent.id());
    const auto neighbors = routing_graph->besides(parent); // TODO(zusizusiz): can not get next lanelets 
    // check the size
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_velocity_planner"),
      "Parent lanelet %ld has %zu neighbors", parent.id(), neighbors.size());
    for (const auto & neighbor : neighbors) parent_neighbors.insert(neighbor.id());
  }
  std::set<lanelet::Id> associative_roundabout_lanelets;
  associative_roundabout_lanelets.insert(lane.id());
  for (const auto & parent_neighbor_id : parent_neighbors) {
    RCLCPP_WARN(
      rclcpp::get_logger("behavior_velocity_planner"),
      "Parent neighbor lanelet found: %ld", parent_neighbor_id);
    const auto parent_neighbor = lanelet_map->laneletLayer.get(parent_neighbor_id);
    const auto followings = routing_graph->following(parent_neighbor);
    for (const auto & following : followings) {
      if (following.attributeOr("roundabout", "else") == roundabout_attr) {
        associative_roundabout_lanelets.insert(following.id());
        RCLCPP_WARN(
          rclcpp::get_logger("behavior_velocity_planner"),
          "Associative roundabout lanelet found: %ld", following.id());
      }
    }
  }
  return associative_roundabout_lanelets;
}

bool RoundaboutModuleManager::hasSameParentLaneletAndTurnDirectionWithRegistered(
  const lanelet::ConstLanelet & lane) const
{
  for (const auto & scene_module : scene_modules_) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    const auto & associative_ids = roundabout_module->getAssociativeIds();
    if (associative_ids.find(lane.id()) != associative_ids.end()) {
      return true;
    }
  }
  return false;
}

void RoundaboutModuleManager::sendRTC(const Time & stamp)
{
  double min_distance = std::numeric_limits<double>::infinity();
  std_msgs::msg::String decision_type;

  for (const auto & scene_module : scene_modules_) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    const UUID uuid = getUUID(scene_module->getModuleId());
    updateRTCStatus(
      uuid, scene_module->isSafe(), State::RUNNING, scene_module->getDistance(), stamp);

    // ==========================================================================================
    // module debug data
    // ==========================================================================================
    const auto internal_debug_data = roundabout_module->getInternalDebugData();
    if (internal_debug_data.distance < min_distance) {
      min_distance = internal_debug_data.distance;
    }
    decision_type.data += (internal_debug_data.decision_type + "\n");
  }
  rtc_interface_.publishCooperateStatus(stamp);  // publishRTCStatus()

  // ==========================================================================================
  // publish module debug data
  // ==========================================================================================
  decision_state_pub_->publish(decision_type);
}

void RoundaboutModuleManager::modifyPathVelocity(
  autoware_internal_planning_msgs::msg::PathWithLaneId * path)
{
  SceneModuleManagerInterfaceWithRTC::modifyPathVelocity(path);
}

void RoundaboutModuleManager::setActivation()
{
  for (const auto & scene_module : scene_modules_) {
    const auto roundabout_module = std::dynamic_pointer_cast<RoundaboutModule>(scene_module);
    scene_module->setActivation(rtc_interface_.isActivated(getUUID(scene_module->getModuleId())));
    scene_module->setRTCEnabled(rtc_interface_.isRTCEnabled(getUUID(scene_module->getModuleId())));
  }
}

void RoundaboutModuleManager::deleteExpiredModules(
  const autoware_internal_planning_msgs::msg::PathWithLaneId & path)
{
  const auto isModuleExpired = getModuleExpiredFunction(path);

  auto itr = scene_modules_.begin();
  while (itr != scene_modules_.end()) {
    if (isModuleExpired(*itr)) {
      // default
      removeRTCStatus(getUUID((*itr)->getModuleId()));
      removeUUID((*itr)->getModuleId());
      registered_module_id_set_.erase((*itr)->getModuleId());
      itr = scene_modules_.erase(itr);
    } else {
      itr++;
    }
  }
}

}  // namespace autoware::behavior_velocity_planner

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::behavior_velocity_planner::RoundaboutModulePlugin,
  autoware::behavior_velocity_planner::PluginInterface)
