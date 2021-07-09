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

#include "social_lib.h"
#include "ast/ast.hpp"
#include "ast/parsing.hpp"
#include "amrl_msgs/Pose2Df.h"
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
#include "amrl_msgs/SocialPipsSrv.h"
#include "amrl_msgs/HumanStateMsg.h"
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
using amrl_msgs::HumanStateMsg;
using amrl_msgs::SocialPipsSrv;
using std_msgs::Bool;
using std_msgs::String;
using std::max;
using std::min;
using std::cout;
using std::endl;
using std::ofstream;
using geometry_msgs::Pose2D;
using geometry::Angle;
using std_msgs::String;
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
DEFINE_string(ast_path,  "expers/nice_dipr/", "Path to synthesized predicates.");

bool run_ = true;

// Globals for a quick and dirty controller
// bool nav_complete_ = false;
// bool have_dynamics_ = false;
// bool have_nav_stats_ = false;
// bool have_localization_ = false;
bool target_locked_ = false;
bool have_doors_ = false;
// int target_  = 0;
string state_ = "GoAlone";
string last_state_ = "GoAlone";
Vector2f pose_(0, 0);
float theta_ = 0;
Vector2f local_target_(0, 0);
Vector2f vel_(0, 0);
Vector2f goal_pose_(3.98, 8.855);
vector<HumanStateMsg> human_states_ = {};
vector<Pose2Df> human_poses_;
HumanStateMsg human_a_;
HumanStateMsg human_b_;
HumanStateMsg human_c_;
vector<json> demos_ = {};

// Publishers
ros::Publisher halt_pub_;
ros::Publisher go_alone_pub_;
ros::Publisher follow_pub_;
ros::Publisher config_pub_;
ros::Publisher viz_pub_;
ros::Publisher door_pub_;
ros::Publisher local_pub_;
ros::Publisher state_pub_;


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

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

Vector2f VecFromMsg(const amrl_msgs::Pose2Df& msg) {
  return {msg.x, msg.y};
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
        if (pose.norm() < geometry::kEpsilon) {
          continue;
        }
        json h_json;
        const Vector2f transformed = ToRobotFrameP(pose);
        h_json["pose"] = {transformed.x(), transformed.y()};
        humans.push_back(h_json);
    }
    json output = humans;
    return output;
}

json GetHumanJson(const SocialPipsSrv::Request& req) {
    vector<json> humans;
    human_poses_ = req.human_poses;
    for (const amrl_msgs::Pose2Df& human : req.human_poses) {
        const Vector2f pose(human.x, human.y);
        json h_json;
        h_json["pose"] = {pose.x(), pose.y()};
        // 0 Pose is used for an empty human in training data
        if (pose.norm() >= geometry::kEpsilon) {
          humans.push_back(h_json);
        }
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
    const Vector2f diff = h_pose;
    const float dist = diff.norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_human = human;
    }
  }
  return best_human;
}

void WriteDemos() {
  ofstream output_file;
  const string output_name = "social_ref.json";
  const json output = demos_;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

// TODO(jaholtz) Is StraightFreePath Length sufficient, or do we need arcs?
float StraightFreePathLength(const Vector2f& start, const Vector2f& end) {
  //TODO(jaholtz) need to set these to sane defaults (copy from sim)
  float kRobotLength = 1.0;
  float kRearAxleOffset = 0.0;
  float kObstacleMargin = 0.2;
  float kRobotWidth = 0.44;

  if (false) {
    kRobotLength = 0.4;
    kRearAxleOffset = 0.0;
    kObstacleMargin = 0.5;
    kRobotWidth = 0.4;
  }

  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * kRobotLength - kRearAxleOffset + kObstacleMargin;
  // const float l = 0;
  // The robot's half-width.
  const float w = 0.5 * kRobotWidth + kObstacleMargin;

  const Vector2f path = end;
  float free_path_length = path.norm();

  for (const amrl_msgs::Pose2Df& human : human_poses_) {
       const Vector2f pose(human.x, human.y);
    // Transform pose to start reference frame;
    const Vector2f p = pose;
    // If outside width, or behind robot, skip
    //
    if (fabs(p.y()) > w || p.x() <= 0.0f) continue;

    // Calculate distance and store if shorter.
    free_path_length = min(free_path_length, p.x() - l);
  }
  if (free_path_length == path.norm()) {
      free_path_length = 9999;
  }
  if (free_path_length < 0.0) {
    free_path_length = 0;
  }
  // free_path_length = max(0.0f, free_path_length);
  return free_path_length;
}

bool ShouldGoAlone() {
  // Check if Path blocked by unmapped obstacle (humans for now)
  if (StraightFreePathLength({0,0}, local_target_) < 9999) {
    return false;
  }
  return true;
}

bool ShouldFollow() {
  // If the closest robot is moving in the right direction, follow it.
  const HumanStateMsg target = human_a_;
  Vector2f closest_vel(target.translational_velocity.x,
                       target.translational_velocity.y);
  // Simply add the magnitude to relative velocity (may want to follow a
  // human moving slowly towards the goal).
  closest_vel = closest_vel + vel_;
  const Vector2f path = local_target_;
  const Vector2f target_pose(target.pose.x, target.pose.y);
  const Vector2f distance = target_pose;
  const float goal_angle = Angle(path);
  const float closest_angle = Angle(closest_vel);

  if (fabs(AngleDiff(goal_angle, closest_angle)) <=0.5 && distance.norm() > 0.1
      && closest_vel.norm() > 0.1) {
    return true;
  }
  return false;
}

string Transition() {
  if (ShouldGoAlone()) {
    return "GoAlone";
  } else if (ShouldFollow()) {
    return "Follow";
  }
  return "Halt";
}

Example MakeDemo(const json& demo) {
  return JsonToExample(demo);
}

bool ActionRequestCb(SocialPipsSrv::Request &req,
                     SocialPipsSrv::Response &res) {
  if (req.robot_poses.size() < 1) {
    res.action = 0;
    return true;
  }

  // Convert the req to the appropriate form of a demo
  social_lib::GetRelevantHumans(req, &human_a_, &human_b_, &human_c_);
  const auto robot_vel = req.robot_vels[0];
  vel_ = Vector2f(robot_vel.x, robot_vel.y);
  GetHumanJson(req);
  // This check is to guarantee running default behavior if no humans
  // are visible.
  if (req.human_poses.size() > 0) {
    // Transition based on the demo
    state_ = Transition();
  } else {
    state_ = "GoAlone";
    if (state_ != last_state_) {
      cout << "Action: " << "No Humans -> Using Default Nav" << endl;
    }
  }
  if (state_ != last_state_) {
    cout << "Action: " << state_ << endl;
  }
  const json demo = social_lib::DemoFromRequest(req, state_);
  const Example example = MakeDemo(demo);
  demos_.push_back(demo);

  local_target_ = VecFromMsg(req.local_target);
  goal_pose_ = VecFromMsg(req.goal_pose);
  last_state_ = state_;

  // Convert State to Number
  int state = 0;
  if (state_ == "Halt") {
    state = 1;
  } else if (state_ == "Follow") {
    state = 2;
  } else if (state_ == "Pass") {
    state = 3;
  }

  res.action = state;
  return true;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  // signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "social_ref");
  ros::NodeHandle n;

  ros::ServiceServer utmrsActionRequest =
    n.advertiseService("SocialPipsSrv", ActionRequestCb);

  ros::spin();
  WriteDemos();
  return 0;
}
