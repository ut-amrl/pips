//======================================================================== This software is free: you can redistribute it and/or modify
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
#include "ut_multirobot_sim/Pose2Df.h"
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
// using ut_multirobot_sim::utmrsActionReques;
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
bool local_humans_ = false;
// int target_  = 0;
string state_ = "GoAlone";
string last_state_ = "GoAlone";
Vector2f pose_(0, 0);
float theta_ = 0;
Vector2f local_target_(0, 0);
Vector2f vel_(0, 0);
Vector2f goal_pose_(3.98, 8.855);
vector<HumanStateMsg> human_states_ = {};
HumanStateMsg front_;
HumanStateMsg front_left_;
HumanStateMsg front_right_;
HumanStateMsg left_;
HumanStateMsg right_;
vector<json> demos = {};
vector<std::pair<string, string>> trans_list_;

// Publishers
ros::Publisher halt_pub_;
ros::Publisher go_alone_pub_;
ros::Publisher follow_pub_;
ros::Publisher config_pub_;
ros::Publisher viz_pub_;
ros::Publisher door_pub_;
ros::Publisher local_pub_;
ros::Publisher state_pub_;

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
    for (const amrl_msgs::Pose2Df& human : req.human_poses) {
        const Vector2f pose(human.x, human.y);
        json h_json;
        h_json["pose"] = {pose.x(), pose.y()};
        if (pose.norm() < geometry::kEpsilon) {
          continue;
        }
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

void GetRelevantHumans(SocialPipsSrv::Request &req) {
  // Order is front_left, front, front_right
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

  // Assuming 1 robot
  pose_ = VecFromMsg(req.robot_poses[0]);
  for (size_t i = 0; i < req.human_poses.size(); ++i) {
    const amrl_msgs::Pose2Df pose = req.human_poses[i];
    const amrl_msgs::Pose2Df vel = req.human_vels[i];
    HumanStateMsg human;
    human.id = i;
    human.pose.x = pose.x;
    human.pose.y = pose.y;
    human.pose.theta = pose.theta;
    human.translational_velocity.x = vel.x;
    human.translational_velocity.y = vel.y;
    human.translational_velocity.z = 0;
    human.rotational_velocity = vel.theta;
    const Vector2f h_pose(pose.x, pose.y);
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
}

void WriteDemos() {
  ofstream output_file;
  const string output_name = "mpdm_synth_demo.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

string Transition(const Example& example) {
  for (const std::pair<string, string>& trans : trans_list_) {
    if (state_ == trans.first) {
      // Find the appropriate predicate to match the list
      AST::ast_ptr pred;
      if (trans.first == "Halt" && trans.second == "GA") {
        pred = halt_to_ga;
      } else if (trans.first == "Halt" && trans.second == "Follow") {
        pred = halt_to_follow;
      } else if (trans.first == "Halt" && trans.second == "Follow") {
        pred = halt_to_follow;
      } else if (trans.first == "Halt" && trans.second == "Pass") {
        pred = halt_to_pass;
      } else if (trans.first == "Follow" && trans.second == "Halt") {
        pred = follow_to_halt;
      } else if (trans.first == "Follow" && trans.second == "GA") {
        pred = follow_to_ga;
      } else if (trans.first == "Follow" && trans.second == "Pass") {
        pred = follow_to_pass;
      } else if (trans.first == "GA" && trans.second == "Pass") {
        pred = ga_to_pass;
      } else if (trans.first == "GA" && trans.second == "Follow") {
        pred = ga_to_follow;
      } else if (trans.first == "GA" && trans.second == "Halt") {
        pred = ga_to_halt;
      } else if (trans.first == "Pass" && trans.second == "Halt") {
        pred = pass_to_halt;
      } else if (trans.first == "Pass" && trans.second == "GA") {
        pred = pass_to_ga;
      } else if (trans.first == "Pass" && trans.second == "Follow") {
        pred = pass_to_follow;
      }

      if (InterpretBool(pred, example)) {
        return trans.second;
      }
    }
    return last_state_;
  }
  // Halts
  if (state_ == "Halt" &&
      InterpretBool(halt_to_ga, example)) {
    cout << "Halt -> GA" << endl;
    cout << halt_to_ga << endl;
    return "GoAlone";
  }
  if (state_ == "Halt" &&
      InterpretBool(halt_to_follow, example)) {
    cout << "Halt -> Follow" << endl;
    cout << halt_to_follow << endl;
    return "Follow";
  }
  if (state_ == "Halt" &&
      InterpretBool(halt_to_pass, example)) {
    cout << "Halt -> Pass" << endl;
    cout << halt_to_pass << endl;
    return "Pass";
  }
  if (state_ == "Halt") {
    return "Halt";
  }
  // if (state_ == "Halt" &&
      // InterpretBool(halt_to_halt, example)) {
    // cout << "Halt -> Halt" << endl;
    // cout << halt_to_halt << endl;
    // return "Halt";
  // }
  // Follows
  if (state_ == "Follow" &&
      InterpretBool(follow_to_ga, example)) {
    cout << "Follow -> GA" << endl;
    cout << follow_to_ga << endl;
    return "GoAlone";
  }
  if (state_ == "Follow" &&
      InterpretBool(follow_to_halt, example)) {
    cout << "Follow -> Halt" << endl;
    cout << follow_to_halt << endl;
    return "Halt";
  }
  if (state_ == "Follow" &&
      InterpretBool(follow_to_pass, example)) {
    cout << "Follow -> Pass" << endl;
    cout << follow_to_pass << endl;
    return "Pass";
  }
  if (state_ == "Follow") {
    return "Follow";
  }
  // if (state_ == "Follow" &&
      // InterpretBool(follow_to_follow, example)) {
    // cout << "Follow -> Follow" << endl;
    // cout << follow_to_follow << endl;
    // return "Follow";
  // }
  // GoAlones
  cout << "GA -> Halt" << endl;
  cout << ga_to_halt << endl;
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_halt, example)) {
    return "Halt";
  }
  cout << "GA -> Follow" << endl;
  cout << ga_to_follow << endl;
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_follow, example)) {
    return "Follow";
  }
  if (state_ == "GoAlone" &&
      InterpretBool(ga_to_pass, example)) {
    cout << "GA -> Pass" << endl;
    cout << ga_to_pass << endl;
    return "Pass";
  }
  if (state_ == "GoAlone") {
    return "GoAlone";
  }
  // if (state_ == "GoAlone" &&
      // InterpretBool(ga_to_ga, example)) {
    // cout << "GA -> GA" << endl;
    // cout << ga_to_ga << endl;
    // return "GoAlone";
  // }
  // Passes
  cout << "Pass -> GA" << endl;
  cout << pass_to_ga << endl;
  if (state_ == "Pass" &&
      InterpretBool(pass_to_ga, example)) {
    return "GoAlone";
  }
  cout << "Pass -> Follow" << endl;
  cout << pass_to_follow << endl;
  if (state_ == "Pass" &&
      InterpretBool(pass_to_follow, example)) {
    return "Follow";
  }
  if (state_ == "Pass") {
    return "Pass";
  }
  // if (state_ == "Pass" &&
      // InterpretBool(pass_to_pass, example)) {
    // cout << "Pass -> Pass" << endl;
    // cout << pass_to_pass << endl;
    // return "Pass";
  // }
  cout << "Nothing Matched" << endl;
  return "Halt";
}

json DemoFromRequest(const SocialPipsSrv::Request& req) {
  json demo;

  // Input and output states
  string state = "GoAlone";
  if (req.robot_state == 1) {
    state = "Halt";
  } else if (req.robot_state == 2) {
    state = "Follow";
  } else if (req.robot_state == 3) {
    state = "Pass";
  }
  demo["start"] = MakeEntry("start", last_state_);
  demo["output"] = MakeEntry("output", state);
  last_state_ = state;

  demo["door_state"] = MakeEntry("DoorState", req.door_state, {0, 0, 0});
  demo["door_pose"] = MakeEntry("DoorPose",
      {req.door_pose.x, req.door_pose.y}, {1, 0, 0});

  local_target_ = VecFromMsg(req.local_target);
  demo["target"] = MakeEntry("target",
      local_target_, {1, 0, 0});

  goal_pose_ = VecFromMsg(req.goal_pose);
  demo["goal"] = MakeEntry("goal",
      ToRobotFrameP(goal_pose_), {1, 0, 0});

  // Special Humans
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

  // All Humans in a vector
  demo["human_states"] = GetHumanJson(req);

  return demo;
}

void LoadTransitionList(const string& path) {
  std::ifstream infile("path");
  string start, output;
  while(infile >> start >> output) {
    const std::pair<string, string> trans(start, output);
    trans_list_.push_back(trans);
  }
}

Example MakeDemo(const SocialPipsSrv::Request& req) {
  Example example;
  json demo = DemoFromRequest(req);
  return JsonToExample(demo);
}

bool ActionRequestCb(SocialPipsSrv::Request &req,
                     SocialPipsSrv::Response &res) {
  if (req.robot_poses.size() < 1) {
    res.action = 0;
    return true;
  }
  // Get Relevant Humans
  GetRelevantHumans(req);

  // Convert the req to the appropriate form of a demo
  const Example example = MakeDemo(req);
  // Transition based on the demo
  state_ = Transition(example);
  int state = 0;
  cout << "state_: " << state_ << endl;
  if (state_ == "Halt") {
    state = 1;
  } else if (state_ == "Follow") {
    state = 2;
  } else if (state_ == "Pass") {
    state = 3;
  }

  res.action = state;
  // res.follow_target = follow_target if follow / pass
  return true;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "social_interp", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  LoadTransitionList(FLAGS_ast_path + "transitions.txt");
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

  ros::ServiceServer utmrsActionRequest =
    n.advertiseService("SocialPipsSrv", ActionRequestCb);

  ros::spin();
  return 0;
}
