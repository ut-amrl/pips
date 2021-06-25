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

#include <math.h>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "policy/social_lib.h"

using amrl_msgs::Pose2Df;
using amrl_msgs::NavigationConfigMsg;
using amrl_msgs::HumanStateMsg;
using amrl_msgs::SocialPipsSrv;
using Eigen::Vector2f;
using nlohmann::json;
using std::vector;
using std::string;
using std::max;
using std::min;
using std::cout;
using std::endl;
using std::ofstream;
using geometry_msgs::Pose2D;
using geometry::Angle;
using math_util::AngleDiff;
using math_util::AngleDiff;
using math_util::DegToRad;

namespace social_lib {

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

json GetHumanJson(const vector<HumanStateMsg> human_states) {
    vector<json> humans;
    for (HumanStateMsg human : human_states) {
        const Vector2f pose(human.pose.x, human.pose.y);
        if (pose.norm() < geometry::kEpsilon) {
          continue;
        }
        json h_json;
        h_json["pose"] = {pose.x(), pose.y()};
        humans.push_back(h_json);
    }
    return humans;
}

json GetHumanJson(const SocialPipsSrv::Request& req) {
    vector<json> humans;
    for (const amrl_msgs::Pose2Df& human : req.human_poses) {
        const Vector2f pose(human.x, human.y);
        json h_json;
        h_json["pose"] = {pose.x(), pose.y()};
        // 0 Pose is used for an empty human in training data
        if (pose.norm() >= geometry::kEpsilon) {
          humans.push_back(h_json);
        }
    }
    return humans;
}

HumanStateMsg GetClosest(const vector<HumanStateMsg>& humans) {
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

void GetRelevantHumans(const SocialPipsSrv::Request& req,
                       HumanStateMsg* front_human,
                       HumanStateMsg* front_l,
                       HumanStateMsg* front_r) {
  // Order is front_left, front, front_right
  vector<HumanStateMsg> left;
  vector<HumanStateMsg> front_left;
  vector<HumanStateMsg> front; vector<HumanStateMsg> front_right;
  vector<HumanStateMsg> right;

  const float kRobotLength = 0.5;
  const float kLowerLeft = DegToRad(90.0);
  const float kUpperLeft = DegToRad(15.0);
  const float kLowerRight = DegToRad(270.0);
  const float kUpperRight = DegToRad(345.0);

  // Assuming 1 robot
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
    if (h_pose.norm() < geometry::kEpsilon) continue;
    const float angle = Angle(h_pose);
    if (h_pose.x() > kRobotLength) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        front_left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        front_right.push_back(human);
      } else if (angle < kUpperLeft || angle > kUpperRight) {
        front.push_back(human);
      }
    } else if (h_pose.x() > 0) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        right.push_back(human);
      }
    }
  }

  *front_l = GetClosest(front_left);
  *front_human = GetClosest(front);
  *front_r = GetClosest(front_right);
}

void WriteDemos(const vector<json>& demos, const string& output_name) {
  ofstream output_file;
  const json output = demos;
  output_file.open(output_name);
  output_file << std::setw(4) << output << std::endl;
  output_file.close();
}

// TODO(jaholtz) Is StraightFreePath Length sufficient, or do we need arcs?
float StraightFreePathLength(const Vector2f& start,
                             const Vector2f& end,
                             const vector<Pose2Df> human_poses) {
  //TODO(jaholtz) need to set these to sane defaults (copy from sim)
  float kRobotLength = 1.0;
  float kRearAxleOffset = 0.0;
  float kObstacleMargin = 0.2;
  float kRobotWidth = 0.44;

  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * kRobotLength - kRearAxleOffset + kObstacleMargin;
  // const float l = 0;
  // The robot's half-width.
  const float w = 0.5 * kRobotWidth + kObstacleMargin;

  const Vector2f path = end;
  float free_path_length = path.norm();

  for (const amrl_msgs::Pose2Df& human : human_poses) {
       const Vector2f pose(human.x, human.y);
    // Transform pose to start reference frame;
    const Vector2f p = pose;
    // If outside width, or behind robot, skip
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

Vector2f ToRobotFrameP(const Vector2f& pose,
                       const Vector2f& robot_pose,
                       const float& robot_theta) {
    // Transform the pose to robot reference frame
    const Vector2f diff = pose - robot_pose;
    Eigen::Rotation2Df rot(-robot_theta);
    return rot * diff;
}

json DemoFromRequest(const SocialPipsSrv::Request& req,
                     const string& action) {
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
  demo["start"] = MakeEntry("start", state);
  demo["output"] = MakeEntry("output", action);

  demo["door_state"] = MakeEntry("DoorState", req.door_state, {0, 0, 0});
  demo["door_pose"] = MakeEntry("DoorPose",
     {req.door_pose.x, req.door_pose.y}, {1, 0, 0});

  const Vector2f local_target = VecFromMsg(req.local_target);
  demo["target"] = MakeEntry("target",
      local_target, {1, 0, 0});

  const Vector2f goal_pose = VecFromMsg(req.goal_pose);
  const float robot_theta = req.robot_poses[0].theta;
  const Vector2f robot_pose = VecFromMsg(req.robot_poses[0]);
  demo["goal"] = MakeEntry("goal",
      ToRobotFrameP(goal_pose, robot_pose, robot_theta), {1, 0, 0});

  // Special Humans
  HumanStateMsg front, left, right;
  GetRelevantHumans(req, &front, &left, &right);
  demo["front_p"] =
      MakeEntry("front_p",
          {front.pose.x, front.pose.y}, {1, 0, 0});
  demo["front_v"] = MakeEntry("front_v",
      {front.translational_velocity.x,
      front.translational_velocity.y},
      {1, -1, 0});
  demo["fLeft_p"] =
      MakeEntry("fLeft_p",
          {left.pose.x, left.pose.y}, {1, 0, 0});
  demo["fLeft_v"] = MakeEntry("fLeft_v",
      {left.translational_velocity.x,
       left.translational_velocity.y},
      {1, -1, 0});
  demo["fRight_p"] =
      MakeEntry("fRight_p",
          {right.pose.x, right.pose.y}, {1, 0, 0});
  demo["fRight_v"] = MakeEntry("fRight_v",
      {right.translational_velocity.x,
       right.translational_velocity.y},
      {1, -1, 0});

  // All Humans in a vector
  demo["human_states"] = GetHumanJson(req);
  return demo;
}
}