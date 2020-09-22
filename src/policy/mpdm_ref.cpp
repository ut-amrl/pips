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
#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <vector>

#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose2D.h"

using amrl_msgs::Pose2Df;
using amrl_msgs::NavigationConfigMsg;
using Eigen::Vector2f;
using std::vector;
using std::string;
using ut_multirobot_sim::HumanStateMsg;
using std_msgs::Bool;
using std_msgs::String;
using geometry_msgs::Pose2D;

// These are here because everything dies without them.
DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");

bool run_ = true;

// Globals for a quick and dirty controller
bool nav_complete_ = false;
bool have_dynamics_ = false;
bool target_locked_ = false;
int target_  = 0;
string state = "GO_ALONE";
string last_state = "GO_ALONE";
Vector2f pose_(0, 0);
float theta_ = 0;
Vector2f local_target_(0, 0);
float local_target_theta_ = 0;
Vector2f vel_(0, 0);
float omega_ = 0;
Vector2f target_pose_(0, 0);
float target_theta_ = 0;
vector<HumanStateMsg> human_states_ = {};

// Publishers
ros::Publisher halt_pub_;
ros::Publisher go_alone_pub_;
ros::Publisher follow_pub_;
ros::Publisher config_pub_;

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
}

void NavStatusCb(const amrl_msgs::NavStatusMsg msg) {
  nav_complete_ = msg.nav_complete;
  local_target_ = Vector2f(msg.local_target.x, msg.local_target.y);
  vel_ = Vector2f(msg.velocity.x, msg.velocity.y);
}

void HumanStateCb(const ut_multirobot_sim::HumanStateArrayMsg msg) {
  human_states_ = msg.human_states;
  have_dynamics_ = true;
}

void Halt() {
  Bool halt_message;
  halt_message.data = true;
  halt_pub_.publish(halt_message);
}

void GoAlone() {
  Pose2Df target_message;
  target_message.x = target_pose_.x();
  target_message.y = target_pose_.y();
  target_message.theta = target_theta_;
  go_alone_pub_.publish(target_message);
}

void FindTarget() {
  int best_target = 0;
  float best_dist = 9999;
  for (size_t i = 0; i < human_states_.size(); ++i) {
    const HumanStateMsg human = human_states_[i];
    const Vector2f h_pose(human.pose.x, human.pose.y);
    const Vector2f diff = h_pose - pose_;
    const float dist = diff.norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_target = i;
    }
  }
  target_ = best_target;
}

void Follow() {
  if (!target_locked_) {
    FindTarget();
    target_locked_ = true;
  }
  HumanStateMsg target = human_states_[target_];
  const Vector2f target_vel(target.translational_velocity.x,
      target.translational_velocity.y);
  NavigationConfigMsg conf_msg;
  conf_msg.max_vel = target_vel.norm();
  config_pub_.publish(conf_msg);
  Pose2D follow_msg;
  follow_msg.x = target.pose.x;
  follow_msg.y = target.pose.y;
  follow_pub_.publish(follow_msg);
}

void Run() {
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

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
  follow_pub_ = n.advertise<Pose2D>("/nav_override", 1);
  config_pub_ = n.advertise<NavigationConfigMsg>("/nav_config", 1);

  ros::Rate loop(30.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    Run();
    loop.sleep();
  }
  return 0;
}
