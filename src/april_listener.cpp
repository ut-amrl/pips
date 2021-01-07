// Copyright (c) 2020 Jarrett Holtz, jaholtz@cs.utexas.edu

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <stdio.h>

#include <vector>

#include "eigen3/Eigen/Dense"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

#include "math/math_util.h"
#include "ros/ros_helpers.h"

using Eigen::Vector2f;
using std::vector;
using ut_multirobot_sim::HumanStateMsg;
using ut_multirobot_sim::HumanStateArrayMsg;
using apriltag_ros::AprilTagDetectionArray;
using apriltag_ros::AprilTagDetection;

ros::Publisher human_pub_;
HumanStateMsg last_humans_[10];
bool initialized_ = false;
ros::Time last_time_;

void AprilCb(const AprilTagDetectionArray& msg) {
  ros::Time current_time = ros::Time::now();
  ros::Duration dt = current_time - last_time_;
  HumanStateArrayMsg human_states;
  vector<HumanStateMsg> humans;
  for (const AprilTagDetection& p : msg.detections) {
    const int id = p.id[0];
    HumanStateMsg h;
    h.pose.x = p.pose.pose.pose.position.z;
    h.pose.y = -p.pose.pose.pose.position.y;
    h.pose.theta = 0;
    h.rotational_velocity = 0;
    h.translational_velocity.x = 0;
    h.translational_velocity.y = 0;
    if (initialized_) {
      const HumanStateMsg p0 = last_humans_[id];
      const Vector2f  pose0(p0.pose.x, p0.pose.y);
      const Vector2f  pose1(p.pose.pose.pose.position.x,
                            p.pose.pose.pose.position.y);
      const Vector2f vel = (pose1 - pose0) / dt.toSec();
      h.translational_velocity.x = vel.x();
      h.translational_velocity.y = vel.y();
    }
    // Assuming single tag, not a bundle
    last_humans_[id] = h;
    humans.push_back(h);
    initialized_ = true;
  }
  human_states.human_states = humans;
  human_pub_.publish(human_states);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "laser_segmentation");
  ros::NodeHandle n;
  ros::Subscriber april_sub = n.subscribe("tag_detections", 1, AprilCb);
  human_pub_ = n.advertise<HumanStateArrayMsg>("human_states", 1, false);
  last_time_ = ros::Time::now();

  ros::spin();
  return 0;
}
