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
#include "ut_multirobot_sim/SimulatorStateMsg.h"
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
using ut_multirobot_sim::SimulatorStateMsg;
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
// // Room Above the hallway
DEFINE_double(x,  14.8, "X-Coordinate of target location.");
DEFINE_double(y,  14.1, "Y-Coordinate of target location.");
// Other End of the Hallway
// DEFINE_double(x,  8.8, "X-Coordinate of target location.");
// DEFINE_double(y,  8.7, "Y-Coordinate of target location.");
DEFINE_double(theta,  0.0, "Theta-Coordinate of target location.");

bool run_ = true;
bool state_set_ = false;

// Globals for a quick and dirty controller
SimulatorStateMsg sim_state_;
bool sim_step_ = false;
bool nav_complete_ = false;
bool have_dynamics_ = false;
bool have_nav_stats_ = false;
bool have_localization_ = false;
bool target_locked_ = false;
int pass_target_;
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
  const Vector2f test(msg.local_target.x, msg.local_target.y);
  const Vector2f diff = pose_ - test;
  if (state_ != "Halt" && diff.norm() > geometry::kEpsilon) {
      local_target_ = Vector2f(msg.local_target.x, msg.local_target.y);
  }
  vel_ = Vector2f(msg.velocity.x, msg.velocity.y);
  have_nav_stats_ = true;
}

void HumanStateCb(const ut_multirobot_sim::HumanStateArrayMsg msg) {
  human_states_ = msg.human_states;
  have_dynamics_ = true;
}

void SetStateCb(const std_msgs::String msg) {
  last_state_ = state_;
  state_ = msg.data;
  state_set_ = true;
}

void SimStateCb(const SimulatorStateMsg msg) {
  if (msg.sim_step_count > sim_state_.sim_step_count) {
    sim_step_ = true;
  }
  sim_state_ = msg;
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
  demo["door_state"] = MakeEntry("DoorState", 1, {0, 0, 0});
  // cout << "Local Target: " << local_target_.x() << "," << local_target_.y() << endl;
  demo["target"] = MakeEntry("target",
      ToRobotFrameP(local_target_), {1, 0, 0});
  // cout << "Transformed: " << demo["target"]["value"][0] << "," << demo["target"]["value"][1] << endl;
  cout << "Goal: " << goal_pose_.x() << "," << goal_pose_.y() << endl;
  demo["goal"] = MakeEntry("goal",
      ToRobotFrameP(goal_pose_), {1, 0, 0});
  // demo["free_path_length"] = MakeEntry("free_path",
      // StraightFreePathLength(pose_, local_target_), {1, 0, 0});
  cout << "Goal Transformed: " << demo["goal"]["value"][0] << "," << demo["goal"]["value"][1] << endl;

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

void Run() {
  if (have_localization_ && have_dynamics_ && have_nav_stats_ && state_set_) {
    cout << state_ << endl;
    GetRelevantHumans();
    SaveDemo();
    last_state_ = state_;
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
  ros::Subscriber state_sub =
      n.subscribe("/robot_state", 1, &SetStateCb);
  ros::Subscriber sim_state_sub =
      n.subscribe("/sim_state", 1, &SimStateCb);

  ros::Rate loop(30.0);
  while (run_ && ros::ok()) {
    switch (sim_state_.sim_state) {
      case 1 : {
        ros::spinOnce();
        Run();
        sim_step_ = false;
        loop.sleep();
      } break;
      case SimulatorStateMsg::SIM_STOPPED : {
        ros::spinOnce();
        // Do nothing unless stepping.
        if (sim_step_) {
          Run();
          // Disable stepping until a step message is received.
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
