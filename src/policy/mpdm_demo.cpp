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
#include "ast/ast.hpp"
#include "ast/parsing.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/publisher.h"
#include "cobot_msgs/CobotLocalizationMsg.h"
#include "cobot_msgs/CobotDoorDetectionsMsg.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
#include "ut_multirobot_sim/SimulatorStateMsg.h"
#include "ut_multirobot_sim/DoorArrayMsg.h"
#include "ut_multirobot_sim/DoorStateMsg.h"
#include "ut_multirobot_sim/DoorControlMsg.h"
#include "ut_multirobot_sim/utmrsStepper.h"
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
using AST::Example;
using Eigen::Vector2f;
using nlohmann::json;
using std::vector;
using std::string;
using ut_multirobot_sim::HumanStateMsg;
using ut_multirobot_sim::SimulatorStateMsg;
using ut_multirobot_sim::DoorStateMsg;
using ut_multirobot_sim::DoorArrayMsg;
using ut_multirobot_sim::DoorControlMsg;
using ut_multirobot_sim::utmrsStepper;
using ut_multirobot_sim::utmrsStepperRequest;
using ut_multirobot_sim::utmrsStepperResponse;
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
using math_util::DegToRad;

// These are here because everything dies without them.
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");
DEFINE_bool(debug, false, "Enable Debug Printing");

bool run_ = true;

// Globals for a quick and dirty controller
SimulatorStateMsg sim_state_;
bool sim_step_ = false;
bool local_humans_ = false;
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
vector<ut_multirobot_sim::Pose2Df> human_poses_ = {};
vector<DoorStateMsg> door_states_ = {};
HumanStateMsg front_;
HumanStateMsg front_left_;
HumanStateMsg front_right_;
HumanStateMsg left_;
HumanStateMsg right_;
ros::ServiceClient step_client_;

vector<json> demos = {};

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void SetStateCb(const std_msgs::String& msg) {
  last_state_ = state_;
  state_ = msg.data;
}

void PauseCb(const std_msgs::Bool& msg) {
  sim_state_.sim_state = !sim_state_.sim_state;
}

void StepCb(const std_msgs::Bool& msg) {
  sim_step_ = true;
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

Vector2f VecFromMsg(const amrl_msgs::Pose2Df& msg) {
  return {msg.x, msg.y};
}

Vector2f VecFromMsg(const ut_multirobot_sim::Pose2Df& msg) {
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

json GetHumanJson(const utmrsStepperResponse& req) {
    vector<json> humans;
    human_poses_ = req.human_poses;
    for (const ut_multirobot_sim::Pose2Df& human : req.human_poses) {
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

json DemoFromRequest(const utmrsStepperResponse& req) {
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
  cout << "Robot Pose: " << pose_.x() << ", " << pose_.y() << endl;
  demo["target"] = MakeEntry("target",
      local_target_, {1, 0, 0});
  // cout << "Local Target: " << ToRobotFrameP(local_target_).x() << "," << ToRobotFrameP(local_target_).y() << endl;

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
  demos.push_back(demo);
  return demo;
}

Example MakeDemo(const utmrsStepperResponse& req) {
  Example example;
  json demo = DemoFromRequest(req);
  return JsonToExample(demo);
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

void GetRelevantHumans(utmrsStepperResponse& req) {
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
    const ut_multirobot_sim::Pose2Df pose = req.human_poses[i];
    const ut_multirobot_sim::Pose2Df vel = req.human_vels[i];
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

void Run() {
    cout << "Running Demo Recorder" << endl;
    cout << "State: " << state_ << endl;
    // Convert State to Action
    int action = 0;
    if (state_ == "Halt") {
      action = 1;
    } else if (state_ == "Follow") {
      action = 2;
    } else if (state_ == "Pass") {
      action = 3;
    }
    // Step Gym and Store Observation
    ut_multirobot_sim::utmrsStepperRequest req;
    ut_multirobot_sim::utmrsStepperResponse res;
    req.action = action;
    step_client_.call(req, res);

    // Turn Observation into a demonstration
    GetRelevantHumans(res);
    // Convert the req to the appropriate form of a demo
    const Example example = MakeDemo(res);

}

void WriteDemos() {
  ofstream output_file;
  const string output_name = "social_demo.json";
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "social_demo");
  ros::NodeHandle n;

  // Subscribers
  ros::Subscriber state_sub =
      n.subscribe("/robot_state", 1, &SetStateCb);
  ros::Subscriber pause_sub =
      n.subscribe("/sim_start_stop", 1, &PauseCb);
  ros::Subscriber sim_step_sub =
      n.subscribe("/sim_step", 1, &StepCb);
  step_client_ =
      n.serviceClient<ut_multirobot_sim::utmrsStepper>("utmrsStepper");

  step_client_.waitForExistence();
  cout << "Services Acquired" << endl;

  ros::Rate loop(60.0);
  while (run_ && ros::ok()) {
    switch (sim_state_.sim_state) {
      case SimulatorStateMsg::SIM_RUNNING : {
        ros::spinOnce();
        Run();
        sim_step_ = false;
        loop.sleep();
      } break;
      case SimulatorStateMsg::SIM_STOPPED : {
        ros::spinOnce();
        if (sim_step_) {
          Run();
          sim_step_ = false;
        }
      } break;
      default: {
      }
    }
  }
  WriteDemos();
  return 0;
}
