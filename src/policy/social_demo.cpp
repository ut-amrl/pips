//========================================================================
// This software is free: you can redistribute it and/or modify
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
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================
#include "policy/social_lib.h"

#include <fstream>
#include "ast/ast.hpp"
#include "ast/parsing.hpp"
#include "gflags/gflags.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visitors/interp_visitor.hpp"
#include "visitors/print_visitor.hpp"

using amrl_msgs::Pose2Df;
using amrl_msgs::NavigationConfigMsg;
using Eigen::Vector2f;
using nlohmann::json;
using std::vector;
using std::string;
using amrl_msgs::HumanStateMsg;
using amrl_msgs::SocialPipsSrv;
using std::max;
using std::min;
using std::cout;
using std::endl;
using std::ofstream;
using geometry_msgs::Pose2D;
using geometry::Angle;
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

string state_ = "GoAlone";
string last_state_ = "GoAlone";
vector<std::pair<string, string>> trans_list_;
vector<json> demos_;

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
  const json demo = social_lib::DemoFromRequest(req, state_);
  const Example example = MakeDemo(demo);
  demos_.push_back(demo);

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

void SetStateCb(const std_msgs::String& msg) {
  last_state_ = state_;
  state_ = msg.data;
}

int main(int argc, char** argv) {
  gflags::ParseCommandLineFlags(&argc, &argv, false);


  // Initialize ROS.
  ros::init(argc, argv, "social_interp");
  ros::NodeHandle n;

  ros::Subscriber state_sub =
      n.subscribe("/robot_state", 1, &SetStateCb);
  ros::ServiceServer utmrsActionRequest =
    n.advertiseService("SocialPipsSrv", ActionRequestCb);

  ros::spin();
  social_lib::WriteDemos(demos_, "labeled_demo.json");
  return 0;
}
