//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\author  Jarrett Holtz, (C) 2020
*/
//========================================================================

#include <dlfcn.h>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <nlohmann/json.hpp>
#include <string.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "ast/ast.hpp"
#include "ast/parsing.hpp"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"
#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotDoorDetectionsMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/publisher.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
#include "ut_multirobot_sim/DoorArrayMsg.h"
#include "ut_multirobot_sim/DoorStateMsg.h"
#include "ut_multirobot_sim/DoorControlMsg.h"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"
#include "amrl_shared_lib/math/geometry.h"
#include "amrl_shared_lib/math/math_util.h"
#include "visualization_msgs/Marker.h"

using amrl_msgs::Pose2Df;
using amrl_msgs::NavigationConfigMsg;
using Eigen::Vector2f;
using nlohmann::json;
using std::vector;
using std::string;
using ut_multirobot_sim::HumanStateMsg;
using ut_multirobot_sim::DoorStateMsg;
using ut_multirobot_sim::DoorArrayMsg;
using ut_multirobot_sim::DoorControlMsg;
using std_msgs::Bool;
using std_msgs::String;
using std::max;
using std::min;
using std::cout;
using std::endl;
using std::ofstream;
using geometry_msgs::Pose2D;
using geometry::Angle;
using math_util::AngleDiff;
using AST::Example;
using AST::ast_ptr;
using AST::Interpret;
using math_util::AngleDiff;
using math_util::DegToRad;

// These are here because everything dies without them.
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");
// Room Above the hallway
DEFINE_double(x,  14.8, "X-Coordinate of target location.");
DEFINE_double(y,  14.1, "Y-Coordinate of target location.");
// // Other End of the Hallway
// DEFINE_double(x,  8.8, "X-Coordinate of target location.");
// DEFINE_double(y,  8.7, "Y-Coordinate of target location.");
DEFINE_double(theta,  0.0, "Theta-Coordinate of target location.");
DEFINE_string(ast_path,  "synthd/mpdm_1/", "Path to synthesized predicates.");

bool run_ = true;

// Globals for a quick and dirty controller
bool nav_complete_ = false;
bool have_dynamics_ = false;
bool have_nav_stats_ = false;
bool have_localization_ = false;
bool target_locked_ = false;
bool have_doors_ = false;
bool local_humans_ = true;
int target_  = 0;
string state_ = "GoAlone";
string last_state_ = "GoAlone";
Vector2f pose_(0, 0);
float theta_ = 0;
Vector2f local_target_(0, 0);
float local_goal_theta_ = 0;
Vector2f vel_(0, 0);
float omega_ = 0;
Vector2f goal_pose_(3.98, 8.855);
float goal_theta_ = 0;
vector<HumanStateMsg> human_states_ = {};
vector<DoorStateMsg> door_states_ = {};
HumanStateMsg front_;
HumanStateMsg front_left_;
HumanStateMsg front_right_;
HumanStateMsg left_;
HumanStateMsg right_;
vector<json> demos = {};

// Publishers
ros::Publisher halt_pub_;
ros::Publisher go_alone_pub_;
ros::Publisher follow_pub_;
ros::Publisher config_pub_;
ros::Publisher viz_pub_;
ros::Publisher door_pub_;
ros::Publisher local_pub_;

// Path to the folder containing pieces of the tranisition function.
// Transition Function ASTs
ast_ptr ga_to_ga;
ast_ptr ga_to_follow;
ast_ptr ga_to_halt;
ast_ptr ga_to_pass;
ast_ptr follow_to_ga;
ast_ptr follow_to_follow;
ast_ptr follow_to_halt;
ast_ptr follow_to_pass;
ast_ptr halt_to_ga;
ast_ptr halt_to_follow;
ast_ptr halt_to_halt;
ast_ptr halt_to_pass;
ast_ptr pass_to_pass;
ast_ptr pass_to_ga;
ast_ptr pass_to_halt;
ast_ptr pass_to_follow;


void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void CobotLocalCb(const cobot_msgs::CobotLocalizationMsg msg) {
  pose_ = Vector2f(msg.x, msg.y);
  theta_ = msg.angle;
  have_localization_ = true;
  amrl_msgs::Localization2DMsg output;
  output.map = msg.map;
  output.pose.x = msg.x;
  output.pose.y = msg.y;
  output.pose.theta = msg.angle;
  local_pub_.publish(output);
}

void LocalizationCb(const amrl_msgs::Localization2DMsg msg) {
  pose_ = Vector2f(msg.pose.x, msg.pose.y);
  theta_ = msg.pose.theta;
  have_localization_ = true;
}

void NavStatusCb(const amrl_msgs::NavStatusMsg msg) {
  nav_complete_ = msg.nav_complete;
  if (state_ != "Halt") {
      cout << "Nav Callback" << endl;
      local_target_ = Vector2f(msg.local_target.x, msg.local_target.y);
      have_nav_stats_ = true;
  }
  vel_ = Vector2f(msg.velocity.x, msg.velocity.y);
}

void HumanStateCb(const ut_multirobot_sim::HumanStateArrayMsg msg) {
  human_states_ = msg.human_states;
  have_dynamics_ = true;
}

void CobotDoorCallback(const cobot_msgs::CobotDoorDetectionsMsg msg) {
  door_states_.clear();
  if (msg.doorID.size() > 0) {
    DoorStateMsg door;
    have_doors_ = true;
    const Vector2f door_left(msg.doorX1[0], msg.doorY1[0]);
    const Vector2f door_right(msg.doorX2[0], msg.doorY2[0]);
    const Vector2f door_pose = door_left - door_right;
    door.pose.x = door_pose.x();
    door.pose.y = door_pose.y();
    const float distance = (door_pose - pose_).norm();
    if (msg.doorStatus[0] == 1) {
      door.doorStatus = 2;
    } else if (distance < 2.0 && distance > 0.5) {
      door.doorStatus = 1;
      // Publish Open Message
      DoorControlMsg control_msg;
      control_msg.command = 2;
      door_pub_.publish(control_msg);
    } else {
      door.doorStatus = 0;
      DoorControlMsg control_msg;
      control_msg.command = 3;
      door_pub_.publish(control_msg);
    }
    door_states_.push_back(door);
  }
}

void DoorStateCb(const ut_multirobot_sim::DoorArrayMsg msg) {
  door_states_ = msg.door_states;
  if (door_states_.size() > 0) {
    have_doors_ = true;
    const DoorStateMsg door = door_states_[0];
    const Vector2f door_pose(door.pose.x, door.pose.y);
    const float distance = (door_pose - pose_).norm();
    if (door.doorStatus == 2) {
    } else if (distance < 2.0 && distance > 0.5) {
      // Publish Open Message
      DoorControlMsg control_msg;
      control_msg.command = 2;
      door_pub_.publish(control_msg);
    } else {
      DoorControlMsg control_msg;
      control_msg.command = 3;
      door_pub_.publish(control_msg);
    }
  }
}

void Halt() {
  target_locked_ = false;
  Bool halt_message;
  halt_message.data = true;
  halt_pub_.publish(halt_message);
}

void GoAlone() {
  target_locked_ = false;
  NavigationConfigMsg conf_msg;
  conf_msg.max_vel = 1.0;
  conf_msg.ang_accel = -1;
  conf_msg.max_accel = -1;
  conf_msg.carrot_dist = -1;
  conf_msg.margin = 0.0;
  conf_msg.max_decel = -1;
  conf_msg.clearance_weight = -0.5;
  config_pub_.publish(conf_msg);
  Pose2Df target_message;
  target_message.x = goal_pose_.x();
  target_message.y = goal_pose_.y();
  target_message.theta = goal_theta_;
  go_alone_pub_.publish(target_message);
}

int FindTarget() {
  int best_target = 0;
  float best_dist = 9999;
  for (size_t i = 0; i < human_states_.size(); ++i) {
    const HumanStateMsg human = human_states_[i];
    const Vector2f h_pose(human.pose.x, human.pose.y);
    const Vector2f diff = h_pose - pose_;
    const float dist = diff.norm();
    Eigen::Rotation2Df rot(-theta_);
    const Vector2f transformed = rot * diff;
    if (dist < best_dist && transformed.x() > 0.0) {
      best_dist = dist;
      best_target = i;
    }
  }
  // target_ = best_target;
  return best_target;
}

void Follow() {
  const float kFollowDist = 0.5;
  HumanStateMsg target = front_;
  const Vector2f h_pose(target.pose.x, target.pose.y);
  const Vector2f towards_bot = pose_ - h_pose;
  const Vector2f target_pose = h_pose + kFollowDist * towards_bot.normalized();
  const Vector2f target_vel(target.translational_velocity.x,
      target.translational_velocity.y);
  NavigationConfigMsg conf_msg;
  conf_msg.max_vel = target_vel.norm() - .001;
  conf_msg.ang_accel = -1;
  conf_msg.max_accel = -1;
  conf_msg.carrot_dist = -1;
  conf_msg.margin = -1;
  conf_msg.max_decel = -1;
  conf_msg.clearance_weight = 1.0;
  config_pub_.publish(conf_msg);
  Pose2Df follow_msg;
  follow_msg.x = target_pose.x();
  follow_msg.y = target_pose.y();
  follow_pub_.publish(follow_msg);
}

void Pass() {
  const float kLeadDist = 1.5;
  HumanStateMsg pass_target_ = human_states_[0];
  const Vector2f h_pose(pass_target_.pose.x, pass_target_.pose.y);
  const Vector2f target_vel(pass_target_.translational_velocity.x,
      pass_target_.translational_velocity.y);
  const Vector2f target_pose = h_pose + kLeadDist * target_vel.normalized();
  NavigationConfigMsg conf_msg;
  conf_msg.max_vel = 0.0;
  conf_msg.ang_accel = -1;
  conf_msg.max_accel = -1;
  conf_msg.carrot_dist = -1;
  conf_msg.margin = 0.0;
  conf_msg.max_decel = -1;
  conf_msg.clearance_weight = 0.0;
  config_pub_.publish(conf_msg);
  Pose2Df follow_msg;
  follow_msg.x = target_pose.x();
  follow_msg.y = target_pose.y();
  follow_pub_.publish(follow_msg);
}

void Pass2() {
  target_locked_ = false;
  Pose2Df target_message;
  target_message.x = goal_pose_.x();
  target_message.y = goal_pose_.y();
  target_message.theta = goal_theta_;
  NavigationConfigMsg conf_msg;
  conf_msg.max_vel = 6.0;
  conf_msg.ang_accel = -1;
  conf_msg.max_accel = -1;
  conf_msg.carrot_dist = -1;
  conf_msg.margin = 0.0;
  conf_msg.max_decel = -1;
  conf_msg.clearance_weight = 1.0;
  config_pub_.publish(conf_msg);
  go_alone_pub_.publish(target_message);
}

// TODO(jaholtz) Is StraightFreePath Length sufficient, or do we need arcs?
float StraightFreePathLength(const Vector2f& start, const Vector2f& end) {
  //TODO(jaholtz) need to set these to sane defaults (copy from sim)
  const float kRobotLength = 0.5;
  const float kRearAxleOffset = 0.0;
  const float kObstacleMargin = 0.5;
  const float kRobotWidth = 0.44;

  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * kRobotLength - kRearAxleOffset + kObstacleMargin;
  // const float l = 0;
  // The robot's half-width.
  const float w = 0.5 * kRobotWidth + kObstacleMargin;

  const Vector2f path = end - start;
  const float angle = Angle(path);
  const Eigen::Rotation2Df rot(-angle);
  float free_path_length = path.norm();

  for (const HumanStateMsg human : human_states_) {
    Vector2f pose(human.pose.x, human.pose.y);
    // Transform pose to start reference frame;
    Vector2f p = rot * (pose - start);
    if (local_humans_) {
      p = pose;
    }
    // If outside width, or behind robot, skip
    //
    if (fabs(p.y()) > w || p.x() < 0.0f) continue;
    // cout << "Start: " << start << endl;
    // cout << "End: " << end << endl;
    // cout << "Angle: " << angle << endl;
    // cout << "w: " << w << endl;
    // cout << "l: " << l << endl;
    // cout << "py: " << p.y() << endl;
    // cout << "px: " << p.x() << endl;

    // Calculate distance and store if shorter.
    free_path_length = min(free_path_length, p.x() - l);
  }
  if (free_path_length == path.norm()) {
      free_path_length = 9999;
  }
  // free_path_length = max(0.0f, free_path_length);
  // // cout << free_path_length << endl;
  return free_path_length;
}

bool ShouldGoAlone() {
  // Check if Path blocked by unmapped obstacle (humans for now)
  const Vector2f path = pose_ - local_target_;
  // cout << "Path norm: " << path.norm() << endl;
  if (StraightFreePathLength(pose_, local_target_) < path.norm()) {
    return false;
  }
  return true;
}

bool ShouldFollow() {
  // If the closest robot is moving in the right direction, follow it.
  int target_id = target_;
  if (!target_locked_) {
    target_id = FindTarget();
  }
  HumanStateMsg target = human_states_[target_id];
  const Vector2f closest_vel(target.translational_velocity.x,
      target.translational_velocity.y);
  const Vector2f path = local_target_ - pose_;
  const Vector2f target_pose(target.pose.x, target.pose.y);
  const Vector2f distance = target_pose - pose_;
  const float goal_angle = Angle(path);
  const float closest_angle = Angle(closest_vel);
  if (fabs(AngleDiff(goal_angle, closest_angle)) <=0.8 && distance.norm() > 1.5) {
    target_ = target_id;
    target_locked_ = true;
    return true;
  }
  return false;
}

json MakeEntry(const string& name, const float& value, const vector<int>& dim) {
  json entry;
  entry["name"] = name;
  entry["value"] = value;
  entry["type"] = "NUM";
  entry["dim"] = dim;
  return entry;
}

json MakeEntry(const string& name, const string& value) {
  json entry;
  entry["name"] = name;
  entry["value"] = value;
  entry["type"] = "STATE";
  entry["dim"] = {0, 0, 0};
  return entry;
}

json MakeEntry(const string& name,
    const Vector2f& value,
    const vector<int>& dim) {
  json entry;
  entry["name"] = name;
  entry["value"] = {value.x(), value.y()};
  entry["type"] = "VEC";
  entry["dim"] = dim;
  return entry;
}

Vector2f ToRobotFrameP(const Vector2f pose) {
    // Transform the pose to robot reference frame
    const Vector2f diff = pose - pose_;
    Eigen::Rotation2Df rot(-theta_);
    return rot * diff;
}

Vector2f ToRobotFrameV(const Vector2f vel) {
    // Transform the pose to robot reference frame
    const Vector2f diff = vel - vel_;
    Eigen::Rotation2Df rot(-theta_);
    return rot * diff;
}

json GetHumanJson() {
    vector<json> humans;
    for (HumanStateMsg human : human_states_) {
        const Vector2f pose(human.pose.x, human.pose.y);
        json h_json;
        const Vector2f transformed = ToRobotFrameP(pose);
        h_json["pose"] = {transformed.x(), transformed.y()};
        humans.push_back(h_json);
    }
    json output = humans;
    return output;
}

HumanStateMsg GetClosest(const vector<HumanStateMsg> humans) {
  float best_dist = 9999;
  HumanStateMsg best_human;
  best_human.pose.x = 9999;
  best_human.pose.y = 9999;
  best_human.translational_velocity.x = 0.0;
  best_human.translational_velocity.y = 0.0;
  for (HumanStateMsg human : humans) {
    const Vector2f h_pose(human.pose.x, human.pose.y);
    const Vector2f diff = h_pose - pose_;
    const float dist = diff.norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_human = human;
    }
  }
  return best_human;
}

vector<HumanStateMsg> GetRelevantHumans() {
  // Order is front_left, front, front_right
  vector<HumanStateMsg> output(5);
  vector<HumanStateMsg> left;
  vector<HumanStateMsg> front_left;
  vector<HumanStateMsg> front;
  vector<HumanStateMsg> front_right;
  vector<HumanStateMsg> right;
  // todo(jaholtz) Consider if we need to shrink or grow this margin.
  const float kRobotLength = 0.5;
  const float kLowerLeft = DegToRad(90.0);
  const float kUpperLeft = DegToRad(15.0);
  const float kLowerRight = DegToRad(270.0);
  const float kUpperRight = DegToRad(345.0);

  for (HumanStateMsg human : human_states_) {
    const Vector2f h_pose(human.pose.x, human.pose.y);
    // Transform the pose to robot reference frame
    const Vector2f diff = h_pose - pose_;
    Eigen::Rotation2Df rot(-theta_);
    const Vector2f transformed = rot * diff;
    const float angle = math_util::AngleMod(Angle(diff) - theta_);
    if (transformed.x() > kRobotLength) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        front_left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        front_right.push_back(human);
      } else if (angle < kUpperLeft || angle > kUpperRight) {
        front.push_back(human);
      }
    } else if (transformed.x() > 0) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        right.push_back(human);
      }
    }
  }
  front_left_ = GetClosest(front_left);
  front_ = GetClosest(front);
  front_right_ = GetClosest(front_right);
  left_ = GetClosest(left);
  right_ = GetClosest(right);
  return output;
}

json GetDemo() {
  json demo;

  // Input and output states
  demo["start"] = MakeEntry("start", last_state_);
  demo["output"] = MakeEntry("output", state_);
  // demo["bot_pose"] = MakeEntry("bot_pose", pose_, {1, 0, 0});
  // demo["bot_vel"] = MakeEntry("bot_vel", vel_, {1, -1, 0});
  // demo["desired_vel"] = 2.0;
  if (have_doors_ && door_states_.size() > 0) {
    demo["door_state"] = MakeEntry("DoorState", door_states_[0].doorStatus, {0, 0, 0});
    demo["door_pose"] = MakeEntry("DoorPose",
        ToRobotFrameP({door_states_[0].pose.x, door_states_[0].pose.y}), {1, 0, 0});
  } else {
    demo["door_state"] = MakeEntry("DoorState", 2, {0, 0, 0});
    demo["door_pose"] = MakeEntry("DoorPose", {9999, 9999}, {1, 0, 0});
  }
  demo["target"] = MakeEntry("target",
      ToRobotFrameP(local_target_), {1, 0, 0});
  demo["goal"] = MakeEntry("goal",
      ToRobotFrameP(goal_pose_), {1, 0, 0});
  demo["free_path_length"] = MakeEntry("free_path",
      StraightFreePathLength(pose_, local_target_), {1, 0, 0});

  // Special Humans
  demo["front_p"] =
      MakeEntry("front_p",
          ToRobotFrameP({front_.pose.x, front_.pose.y}), {1, 0, 0});
  demo["front_v"] = MakeEntry("front_v",
      ToRobotFrameV({front_.translational_velocity.x,
                    front_.translational_velocity.y}),
      {1, -1, 0});
  demo["fLeft_p"] =
      MakeEntry("fLeft_p",
          ToRobotFrameP({front_left_.pose.x, front_left_.pose.y}), {1, 0, 0});
  demo["fLeft_v"] = MakeEntry("fLeft_v",
      ToRobotFrameV({front_left_.translational_velocity.x,
                    front_left_.translational_velocity.y}),
      {1, -1, 0});
  demo["fRight_p"] =
      MakeEntry("fRight_p",
          ToRobotFrameP({front_right_.pose.x, front_right_.pose.y}), {1, 0, 0});
  demo["fRight_v"] = MakeEntry("fRight_v",
      ToRobotFrameV({front_right_.translational_velocity.x,
                    front_right_.translational_velocity.y}),
      {1, -1, 0});
  // All Humans in a vector
  demo["human_states"] = GetHumanJson();

  // Need to add door state, let's make it a single door for now.
  return demo;
}

void SaveDemo() {
  json demo;

  // Input and output states
  demo["start"] = MakeEntry("start", last_state_);
  demo["output"] = MakeEntry("output", state_);
  // demo["bot_pose"] = MakeEntry("bot_pose", pose_, {1, 0, 0});
  // demo["bot_theta"] = MakeEntry("theta", theta_, {0, 0, 0});
  // demo["bot_vel"] = MakeEntry("bot_vel", vel_, {1, -1, 0});
  // demo["desired_vel"] = 2.0;
  // Add the actual state of the door to this when it is implemented.
  if (have_doors_ && door_states_.size() > 0) {
    demo["door_state"] = MakeEntry("DoorState", door_states_[0].doorStatus, {0, 0, 0});
    demo["door_pose"] = MakeEntry("DoorPose",
        ToRobotFrameP({door_states_[0].pose.x, door_states_[0].pose.y}), {1, 0, 0});
  } else {
    demo["door_state"] = MakeEntry("DoorState", 2, {0, 0, 0});
    demo["door_pose"] = MakeEntry("DoorPose", {9999, 9999}, {1, 0, 0});
  }
  demo["target"] = MakeEntry("target",
      ToRobotFrameP(local_target_), {1, 0, 0});
  demo["goal"] = MakeEntry("goal",
      ToRobotFrameP(goal_pose_), {1, 0, 0});
  demo["free_path_length"] = MakeEntry("free_path",
      StraightFreePathLength(pose_, local_target_), {1, 0, 0});

  // Special Humans
  if (!local_humans_) {
    demo["front_p"] =
      MakeEntry("front_p",
          ToRobotFrameP({front_.pose.x, front_.pose.y}), {1, 0, 0});
    demo["front_v"] = MakeEntry("front_v",
        ToRobotFrameV({front_.translational_velocity.x,
          front_.translational_velocity.y}),
        {1, -1, 0});
    demo["fLeft_p"] =
      MakeEntry("fLeft_p",
          ToRobotFrameP({front_left_.pose.x, front_left_.pose.y}), {1, 0, 0});
    demo["fLeft_v"] = MakeEntry("fLeft_v",
        ToRobotFrameV({front_left_.translational_velocity.x,
          front_left_.translational_velocity.y}),
        {1, -1, 0});
    demo["fRight_p"] =
      MakeEntry("fRight_p",
          ToRobotFrameP({front_right_.pose.x, front_right_.pose.y}), {1, 0, 0});
    demo["fRight_v"] = MakeEntry("fRight_v",
        ToRobotFrameV({front_right_.translational_velocity.x,
          front_right_.translational_velocity.y}),
        {1, -1, 0});
  } else {
    demo["front_p"] =
      MakeEntry("front_p",
          {front_.pose.x, front_.pose.y}, {1, 0, 0});
    demo["front_v"] = MakeEntry("front_v",
        {front_.translational_velocity.x,
          front_.translational_velocity.y},
        {1, -1, 0});
    demo["fLeft_p"] =
      MakeEntry("fLeft_p",
          {front_left_.pose.x, front_left_.pose.y}, {1, 0, 0});
    demo["fLeft_v"] = MakeEntry("fLeft_v",
        {front_left_.translational_velocity.x,
          front_left_.translational_velocity.y},
        {1, -1, 0});
    demo["fRight_p"] =
      MakeEntry("fRight_p",
          {front_right_.pose.x, front_right_.pose.y}, {1, 0, 0});
    demo["fRight_v"] = MakeEntry("fRight_v",
        {front_right_.translational_velocity.x,
          front_right_.translational_velocity.y},
        {1, -1, 0});
  }
  // All Humans in a vector
  demo["human_states"] = GetHumanJson();

  // Need to add door state, let's make it a single door for now.
  demos.push_back(demo);
}

void WriteDemos() {
  ofstream output_file;
  const string output_name = "mpdm_synth_demo.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

Example MakeDemo() {
  Example example;
  json demo = GetDemo();
  return JsonToExample(demo);
}

string Transition() {
  const Example example = MakeDemo();
  // Halts
  if (state_ == "Halt" &&
      InterpretBool(halt_to_ga, example)) {
    return "GoAlone";
  }
  if (state_ == "Halt" &&
      InterpretBool(halt_to_follow, example)) {
    return "Follow";
  }
  if (state_ == "Halt" &&
      InterpretBool(halt_to_pass, example)) {
    return "Pass";
  }
  if (state_ == "Halt" &&
      InterpretBool(halt_to_halt, example)) {
    return "Halt";
  }
  // Follows
  if (state_ == "Follow" &&
      InterpretBool(follow_to_pass, example)) {
    return "Pass";
  }
  if (state_ == "Follow" &&
      InterpretBool(follow_to_ga, example)) {
    return "GoAlone";
  }
  if (state_ == "Follow" &&
      InterpretBool(follow_to_halt, example)) {
    return "Halt";
  }
  if (state_ == "Follow" &&
      InterpretBool(follow_to_follow, example)) {
    return "Follow";
  }
  // GoAlones
  cout << "GA>H: " << ga_to_halt << endl;
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_halt, example)) {
    return "Halt";
  }
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_follow, example)) {
    return "Follow";
  }
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_pass, example)) {
    return "Pass";
  }
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_ga, example)) {
    return "GoAlone";
  }
  // Passes
  if (state_ == "Pass" &&
      InterpretBool(pass_to_ga, example)) {
    return "GoAlone";
  }
  if (state_ == "Pass" &&
      InterpretBool(pass_to_follow, example)) {
    return "Follow";
  }
  if (state_ == "Pass" &&
      InterpretBool(pass_to_pass, example)) {
    return "Pass";
  }
  return "Halt";
}

void Run() {
  cout << "Run" << endl;
  if (have_localization_ && have_nav_stats_) {
    cout << "Get Runned" << endl;
    GetRelevantHumans();
    SaveDemo();
    last_state_ = state_;
    state_ = Transition();
    if (state_ == "GoAlone") {
      GoAlone();
    } else if (state_ == "Follow") {
      Follow();
    } else if (state_ == "Pass") {
      Pass2();
    } else {
      Halt();
    }
    cout << "State: " << state_ << endl;
  } else {
      GoAlone();
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "mpdm_ref", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  ga_to_ga = LoadJson(FLAGS_ast_path + "GoAlone_GoAlone.json");
  ga_to_follow = LoadJson(FLAGS_ast_path + "GoAlone_Follow.json");
  ga_to_halt = LoadJson(FLAGS_ast_path + "GoAlone_Halt.json");
  ga_to_pass = LoadJson(FLAGS_ast_path + "GoAlone_Pass.json");
  follow_to_ga = LoadJson(FLAGS_ast_path + "Follow_GoAlone.json");
  follow_to_follow = LoadJson(FLAGS_ast_path + "Follow_Follow.json");
  follow_to_halt = LoadJson(FLAGS_ast_path + "Follow_Halt.json");
  follow_to_pass = LoadJson(FLAGS_ast_path + "Follow_Pass.json");
  halt_to_ga = LoadJson(FLAGS_ast_path + "Halt_GoAlone.json");
  halt_to_follow = LoadJson(FLAGS_ast_path + "Halt_Follow.json");
  halt_to_halt = LoadJson(FLAGS_ast_path + "Halt_Halt.json");
  halt_to_pass = LoadJson(FLAGS_ast_path + "Halt_Pass.json");
  pass_to_ga = LoadJson(FLAGS_ast_path + "Pass_GoAlone.json");
  pass_to_follow = LoadJson(FLAGS_ast_path + "Pass_Follow.json");
  pass_to_pass = LoadJson(FLAGS_ast_path + "Pass_Pass.json");
  pass_to_halt = LoadJson(FLAGS_ast_path + "Pass_Halt.json");

  // Set target
  goal_pose_ = Vector2f(FLAGS_x, FLAGS_y);
  goal_theta_ = FLAGS_theta;

  // Subscribers
  ros::Subscriber status_sub =
      n.subscribe("/nav_status", 1, &NavStatusCb);
  ros::Subscriber cobot_sub =
      n.subscribe("/Cobot/Localization", 1, &CobotLocalCb);
  ros::Subscriber localization_sub =
      n.subscribe("/localization", 1, &LocalizationCb);
  ros::Subscriber human_sub =
      n.subscribe("/human_states", 1, &HumanStateCb);
  ros::Subscriber door_sub =
      n.subscribe("/door_states", 1, &DoorStateCb);
  ros::Subscriber cobot_door_sub =
      n.subscribe("/Cobot/DoorDetector", 1, &CobotDoorCallback);

  // Publishers
  halt_pub_ = n.advertise<Bool>("/halt_robot", 1);
  go_alone_pub_ = n.advertise<Pose2Df>("/move_base_simple/goal", 1);
  follow_pub_ = n.advertise<Pose2Df>("/nav_override", 1);
  config_pub_ = n.advertise<NavigationConfigMsg>("/nav_config", 1);
  viz_pub_ = n.advertise<visualization_msgs::Marker>("vis_marker", 0);
  door_pub_ = n.advertise<DoorControlMsg>("/door/command", 1);
  local_pub_ = n.advertise<amrl_msgs::Localization2DMsg>("/localization", 1);

  ros::Rate loop(30.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    Run();
    loop.sleep();
  }
  WriteDemos();
  return 0;
}
