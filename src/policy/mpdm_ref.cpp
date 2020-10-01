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

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Rotation2D.h>
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <nlohmann/json.hpp>
#include <string.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/publisher.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
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

// These are here because everything dies without them.
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
// // Room Above the hallway
// DEFINE_double(x,  12.3, "X-Coordinate of target location.");
// DEFINE_double(y,  14.1, "Y-Coordinate of target location.");
// Other End of the Hallway
DEFINE_double(x,  8.8, "X-Coordinate of target location.");
DEFINE_double(y,  8.7, "Y-Coordinate of target location.");
DEFINE_double(theta,  0.0, "Theta-Coordinate of target location.");

bool run_ = true;

// Globals for a quick and dirty controller
bool nav_complete_ = false;
bool have_dynamics_ = false;
bool have_nav_stats_ = false;
bool have_localization_ = false;
bool target_locked_ = false;
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
vector<json> demos = {};

// Publishers
ros::Publisher halt_pub_;
ros::Publisher go_alone_pub_;
ros::Publisher follow_pub_;
ros::Publisher config_pub_;
ros::Publisher viz_pub_;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCb(const amrl_msgs::Localization2DMsg msg) {
  pose_ = Vector2f(msg.pose.x, msg.pose.y);
  theta_ = msg.pose.theta;
  have_localization_ = true;
}

void NavStatusCb(const amrl_msgs::NavStatusMsg msg) {
  nav_complete_ = msg.nav_complete;
  if (state_ != "Halt") {
      local_target_ = Vector2f(msg.local_target.x, msg.local_target.y);
  }
  vel_ = Vector2f(msg.velocity.x, msg.velocity.y);
  have_nav_stats_ = true;
}

void HumanStateCb(const ut_multirobot_sim::HumanStateArrayMsg msg) {
  human_states_ = msg.human_states;
  have_dynamics_ = true;
}

void Halt() {
  target_locked_ = false;
  Bool halt_message;
  halt_message.data = true;
  halt_pub_.publish(halt_message);
}

void GoAlone() {
  target_locked_ = false;
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
  if (!target_locked_) {
    target_ = FindTarget();
    target_locked_ = true;
  }
  HumanStateMsg target = human_states_[target_];
  const Vector2f target_vel(target.translational_velocity.x,
      target.translational_velocity.y);
  NavigationConfigMsg conf_msg;
  // cout << target_vel.norm() << endl;
  conf_msg.max_vel = target_vel.norm() + -0.05;
  conf_msg.ang_accel = -1;
  conf_msg.max_accel = -1;
  conf_msg.carrot_dist = -1;
  conf_msg.margin = -1;
  conf_msg.max_decel = -1;
  config_pub_.publish(conf_msg);
  Pose2Df follow_msg;
  follow_msg.x = target.pose.x;
  follow_msg.y = target.pose.y;
  follow_pub_.publish(follow_msg);
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
    const Vector2f p = rot * (pose - start);
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
  // cout << "Path norm: " << path.norm() << endl;
  if (StraightFreePathLength(pose_, local_target_) < 9999) {
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
  if (fabs(AngleDiff(goal_angle, closest_angle)) <=0.8 && distance.norm() > 2.0) {
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

json GetHumanJson() {
    vector<json> humans;
    for (HumanStateMsg human : human_states_) {
        const Vector2f pose(human.pose.x, human.pose.y);
        json h_json;
        h_json["pose"] = {pose.x(), pose.y()};
        humans.push_back(h_json);
    }
    json output = humans;
    return output;
}

void SaveDemo() {
  json demo;
  const HumanStateMsg target = human_states_[target_];
  const HumanStateMsg closest = human_states_[FindTarget()];
  demo["start"] = MakeEntry("start", last_state_);
  demo["output"] = MakeEntry("output", state_);
  demo["bot_pose"] = MakeEntry("bot_pose", pose_, {1, 0, 0});
  demo["local_target"] = MakeEntry("local_target", local_target_, {1, 0, 0});
  // demo["bot_theta"] = MakeEntry("bot_theta", theta_, {0, 0, 0});
  // demo["bot_vel"] = MakeEntry("bot_vel", vel_, {1, -1, 0});
  // demo["goal_pose"] = MakeEntry("goal_pose", goal_pose_, {1, 0, 0});
  // demo["goal_theta"] = MakeEntry("goal_theta", goal_theta_, {0, 0, 0});
  demo["target_pose"] =
      MakeEntry("target_pose", {target.pose.x, target.pose.y}, {1, 0, 0});
  demo["target_vel"] = MakeEntry("target_vel",
      {target.translational_velocity.x, target.translational_velocity.y},
      {1, -1, 0});
  demo["closest_pose"] =
      MakeEntry("closest_pose", {closest.pose.x, closest.pose.y}, {1, 0, 0});
  demo["closest_vel"] = MakeEntry("closest_vel",
      {closest.translational_velocity.x, closest.translational_velocity.y},
      {1, -1, 0});
  demo["human_states"] = GetHumanJson();
  // TODO(jaholtz) needs to write out all of the human positions, as this
  // is necessary for GetStraightFreePathLength.
  demos.push_back(demo);
}

void WriteDemos() {
  ofstream output_file;
  const string output_name = "mpdm_demo.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

string Transition() {
  if (ShouldGoAlone()) {
    return "GoAlone";
  } else if (ShouldFollow()) {
    return "Follow";
  }
  return "Halt";
}

void Run() {
  if (have_localization_ && have_dynamics_ && have_nav_stats_) {
    last_state_ = state_;
    state_ = Transition();
    SaveDemo();
    cout << "State: " << state_ << endl;
    cout << "Local Target: " << local_target_ << endl << endl;
    if (state_ == "GoAlone") {
      GoAlone();
    } else if (state_ == "Follow") {
      Follow();
    } else {
      Halt();
    }
    // PUblish the target
    if (target_locked_) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = ros::Time();
      marker.ns = "my_namespace";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = human_states_[target_].pose.x;
      marker.pose.position.y = human_states_[target_].pose.y;
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = 1;
      marker.scale.y = 1;
      marker.scale.z = 0.0;
      marker.color.a = 1.0; // Don't forget to set the alpha!
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      viz_pub_.publish( marker );
    }
  }
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "mpdm_ref", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  // Set target
  goal_pose_ = Vector2f(FLAGS_x, FLAGS_y);
  goal_theta_ = FLAGS_theta;

  // Subscribers
  ros::Subscriber status_sub =
      n.subscribe("/nav_status", 1, &NavStatusCb);
  ros::Subscriber localization_sub =
      n.subscribe("/localization", 1, &LocalizationCb);
  ros::Subscriber human_sub =
      n.subscribe("/human_states", 1, &HumanStateCb);

  // Publishers
  halt_pub_ = n.advertise<Bool>("/halt_robot", 1);
  go_alone_pub_ = n.advertise<Pose2Df>("/move_base_simple/goal", 1);
  follow_pub_ = n.advertise<Pose2Df>("/nav_override", 1);
  config_pub_ = n.advertise<NavigationConfigMsg>("/nav_config", 1);
  viz_pub_ = n.advertise<visualization_msgs::Marker>("vis_marker", 0);

  ros::Rate loop(30.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    Run();
    loop.sleep();
  }
  WriteDemos();
  return 0;
}
