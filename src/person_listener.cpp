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
#include "leg_tracker/PersonArray.h"
#include "leg_tracker/Person.h"

#include "math/math_util.h"
#include "ros/ros_helpers.h"

using Eigen::Vector2f;
using std::vector;
using ut_multirobot_sim::HumanStateMsg;
using ut_multirobot_sim::HumanStateArrayMsg;
using leg_tracker::PersonArray;
using leg_tracker::Person;

ros::Publisher human_pub_;
PersonArray last_persons_;
ros::Time last_time_;

int GetLastPerson(const Person& p) {
  for (size_t i = 0; i < last_persons_.people.size(); ++i) {
    if (last_persons_.people[i].id == p.id) {
      return i;
    }
  }
  return -1;
}

void PersonCb(const PersonArray& msg) {
  ros::Time current_time = ros::Time::now();
  ros::Duration dt = current_time - last_time_;
  HumanStateArrayMsg human_states;
  vector<HumanStateMsg> humans;
  for (Person p : msg.people) {
    HumanStateMsg h;
    h.pose.x = p.pose.position.x;
    h.pose.y = p.pose.position.y;
    h.pose.theta = 0;
    h.rotational_velocity = 0;
    h.translational_velocity.x = 0;
    h.translational_velocity.y = 0;
    int last_index = GetLastPerson(p);
    // TODO(jaholtz)
    if (last_index > -1) {
      const Person p0 = last_persons_.people[last_index];
      const Vector2f  pose0(p0.pose.position.x, p0.pose.position.y);
      const Vector2f  pose1(p.pose.position.x, p.pose.position.y);
      const Vector2f vel = (pose1 - pose0) / dt.toSec();
      h.translational_velocity.x = vel.x();
      h.translational_velocity.y = vel.y();
    }
    humans.push_back(h);
  }
  human_states.human_states = humans;
  human_pub_.publish(human_states);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "laser_segmentation");
  ros::NodeHandle n;
  ros::Subscriber human_sub = n.subscribe("people_tracked", 1, PersonCb);
  human_pub_ = n.advertise<HumanStateArrayMsg>("human_states", 1, false);
  last_time_ = ros::Time::now();

  ros::spin();
  return 0;
}
