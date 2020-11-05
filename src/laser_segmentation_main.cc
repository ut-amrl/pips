// Copyright (c) 2020 Joydeep Biswas, joydeepb@cs.utexas.edu

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
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/ColorRGBA.h"
#include "visualization_msgs/Marker.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"

#include "math/math_util.h"
#include "ros/ros_helpers.h"

using Eigen::Vector2f;
using Eigen::Rotation2Df;
using sensor_msgs::LaserScan;
using std::vector;
using visualization_msgs::Marker;
using ut_multirobot_sim::HumanStateMsg;

DECLARE_int32(v);

DEFINE_string(laser, "scan", "Laser scan topic");

Marker vis_msg_;
ros::Publisher vis_pub_;
ros::Publisher human_pub_;
ros::Publisher person_viz_pub_;

DEFINE_double(N, 5, "Min num points per cluster");
DEFINE_double(M, 1000, "Min num points per cluster");
DEFINE_double(theta, 5, "Min theta angle");
DEFINE_double(dist, 5, "Distance threshold");
DEFINE_double(laser_dist, 5, "Distance threshold");

Vector2f human_pose_(0.0 , 0.0);
const float time_step_ = 1.0 / 60.0;

vector<vector<Vector2f>> RunClustering(const vector<Vector2f>& point_cloud) {
  const float kBetaMin = math_util::DegToRad(FLAGS_theta);
  const float kSqDistMax = math_util::Sq(FLAGS_dist);

  vector<vector<Vector2f>> clusters;
  vector<Vector2f> cluster;
  for (size_t i = 0; i + 1 < point_cloud.size(); ++i) {
    const Vector2f& p0 = point_cloud[i];
    const Vector2f& p1 = point_cloud[i + 1];
    const float a_sq = (p0 - p1).squaredNorm();
    const float b_sq = p1.squaredNorm();
    const float c_sq = p0.squaredNorm();
    const float beta = acos((a_sq + b_sq - c_sq) / (2.0f * sqrt(a_sq * b_sq)));
    // if (beta > kBetaMin && a_sq > kSqDistMax) {
    //   printf("a:%f b:%f c:%f %6.1f\u00b0\n",
    //       sqrt(a_sq), sqrt(b_sq), sqrt(c_sq), math_util::RadToDeg(beta));
    // }
    if (cluster.empty()) cluster.push_back(p0);
    if (beta > kBetaMin && beta < M_PI - kBetaMin && a_sq < kSqDistMax && p1.norm() < FLAGS_laser_dist) {
      cluster.push_back(p1);
    } else {
      if (cluster.size() > FLAGS_N) {
        clusters.push_back(cluster);
      }
      cluster.clear();
    }
  }
  return clusters;
}

Vector2f GetCentroid(const vector<Vector2f>& cluster) {
  Vector2f output;
  for (const auto& point : cluster) {
    output += point;
  }
  output = output / cluster.size();
  return output;
}

float GetDensityHack(const vector<Vector2f>& cluster) {
  const int size = cluster.size();
  float length = 0;
  for (size_t i = 0; i < cluster.size(); ++i) {
    for (size_t j = i; j < cluster.size(); ++j) {
      const float dist = (cluster[j] - cluster[i]).norm();
      if (dist > length) {
        length = dist;
      }
    }
  }
  return length / float(size);
}

float GetLength(const vector<Vector2f>& cluster) {
  float length = 0;
  for (size_t i = 0; i < cluster.size(); ++i) {
    for (size_t j = i; j < cluster.size(); ++j) {
      const float dist = (cluster[j] - cluster[i]).norm();
      if (dist > length) {
        length = dist;
      }
    }
  }
  return length;
}

// TODO (Needs an option for no human, and better condition)
vector<Vector2f> GetSingleHuman(vector<vector<Vector2f>> clusters) {
  vector<Vector2f> best_cluster;
  float best_size = 9999;
  for (const auto& clust : clusters) {
    const float d_hack = GetDensityHack(clust);
    const float length = GetDensityHack(clust);
    std::cout << "Density Hack: " << d_hack << std::endl;
    if (length < best_size && clust.size() < FLAGS_M) {
      best_cluster = clust;
      best_size = d_hack;
    }
  }
  std::cout << "Best Hack: " << best_size << std::endl << std::endl;
  std_msgs::ColorRGBA rgb;
  rgb.a = 1.0;
  rgb.b = 1.0;
  rgb.g = 0.0;
  rgb.r = 0.0;
  for (size_t i = 0; i + 1 < best_cluster.size(); ++i) {
    ros_helpers::DrawEigen2DLine(best_cluster[i], best_cluster[i + 1], rgb, &vis_msg_);
  }
  return best_cluster;
}

void PublishHumans(const vector<vector<Vector2f>> clusters) {
  ut_multirobot_sim::HumanStateArrayMsg array_msg;
  HumanStateMsg msg;
  const Vector2f centroid = GetCentroid(GetSingleHuman(clusters));
  msg.pose.x = centroid.x();
  msg.pose.y = centroid.y();
  // Should actually calculate the time
  if (human_pose_.norm() > 0.001) {
    const Vector2f diff = centroid - human_pose_;
    msg.translational_velocity.x = diff.x() / time_step_;
    msg.translational_velocity.y = diff.y() / time_step_;
  } else {
    msg.translational_velocity.x = 0.0;
    msg.translational_velocity.y = 0.0;
  }
  human_pose_ = centroid;
  array_msg.human_states = {msg};
  human_pub_.publish(array_msg);
  // visualization_msgs::Marker marker;
  // marker.header.frame_id = "laser";
  // marker.header.stamp = ros::Time();
  // marker.id = 0;
  // marker.type = visualization_msgs::Marker::SPHERE;
  // marker.action = visualization_msgs::Marker::ADD;
  // marker.pose.position.x = centroid.x();
  // marker.pose.position.y = centroid.y();
  // marker.pose.position.z = 0.0;
  // marker.pose.orientation.x = 0.0;
  // marker.pose.orientation.y = 0.0;
  // marker.pose.orientation.z = 0.0;
  // marker.pose.orientation.w = 1.0;
  // marker.scale.x = 1;
  // marker.scale.y = 1;
  // marker.scale.z = 1;
  // marker.color.a = 1.0; // Don't forget to set the alpha!
  // marker.color.r = 0.0;
  // marker.color.g = 1.0;
  // marker.color.b = 0.0;
  // person_viz_pub_.publish(marker);
}

void LaserCallback(const LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser message: %f\n", msg.header.stamp.toSec());
  }
  static vector<Vector2f> headings_;
  if (headings_.size() != msg.ranges.size()) {
    headings_.resize(msg.ranges.size());
    for (size_t i = 0; i < headings_.size(); ++i) {
      const float a =
          msg.angle_min + msg.angle_increment * static_cast<float>(i);
      headings_[i] = Vector2f(cos(a), sin(a));
    }
  }
  static vector<Vector2f> point_cloud_;
  point_cloud_.resize(msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    point_cloud_[i] = msg.ranges[i] * headings_[i];
  }
  vector<vector<Vector2f>> clusters = RunClustering(point_cloud_);
  vis_msg_.header.frame_id = msg.header.frame_id;
  vis_msg_.header.stamp = msg.header.stamp;
  ros_helpers::ClearMarker(&vis_msg_);
  // auto best = GetSingleHuman(clusters);
  // clusters.clear();
  // clusters.push_back(best);
  PublishHumans(clusters);
  int count = 0;
  // for (const auto& c : clusters) {
    // const Vector2f centroid = GetCentroid(c);
    // // if ((c.size() < FLAGS_M || centroid.norm() < 2.0) && centroid.norm() > 1.0) {
      // std::cout << centroid .norm() << std::endl;
      // count++;
      // for (size_t i = 0; i + 1 < c.size(); ++i) {
        // ros_helpers::DrawEigen2DLine(c[i], c[i + 1], &vis_msg_);
      // }
    // // }
  // }
  std::cout << "Num Clusters: " << count << std::endl;
  if (vis_msg_.points.empty()) return;
  vis_pub_.publish(vis_msg_);
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  ros::init(argc, argv, "laser_segmentation");
  ros::NodeHandle n;
  ros::Subscriber laser_sub = n.subscribe(FLAGS_laser, 1, LaserCallback);
  vis_pub_ = n.advertise<Marker>("laser_segments", 1, false);
  human_pub_ = n.advertise<Marker>("human_states", 1, false);
  person_viz_pub_ = n.advertise<visualization_msgs::Marker>( "person_viz", 0 );
  // cluster_pub_ = n.advertise<Marker>("laser_segments", 1, false);
  ros_helpers::InitRosHeader("velodyne", &vis_msg_.header);
  ros_helpers::SetIdentityRosQuaternion(&vis_msg_.pose.orientation);
  vis_msg_.pose.position.x = 0;
  vis_msg_.pose.position.y = 0;
  vis_msg_.pose.position.z = 0;
  vis_msg_.scale.x = 0.1;
  ros_helpers::SetRosColor(1, 0, 0, 1, &vis_msg_.color);
  vis_msg_.type = Marker::LINE_LIST;
  vis_msg_.action = Marker::ADD;
  vis_msg_.ns = "laser_segmentation";

  ros::spin();
  return 0;
}
