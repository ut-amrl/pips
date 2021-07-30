//========================================================================
//This software is free: you can redistribute it and/or modify
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


#include "policy/social_lib.h"

#include "ast/ast.hpp"
#include "ast/parsing.hpp"
#include "gflags/gflags.h"
#include <fstream>
#include "ros/ros.h"
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

string Transition(const Example& example) {
  for (const std::pair<string, string>& trans : trans_list_) {
    if (state_ == trans.first) {
      // Find the appropriate predicate to match the list
      AST::ast_ptr pred;
      if (trans.first == "Halt" && trans.second == "GoAlone") {
        pred = halt_to_ga;
      } else if (trans.first == "Halt" && trans.second == "Follow") {
        pred = halt_to_follow;
      } else if (trans.first == "Halt" && trans.second == "Pass") {
        pred = halt_to_pass;
      } else if (trans.first == "Follow" && trans.second == "Halt") {
        pred = follow_to_halt;
      } else if (trans.first == "Follow" && trans.second == "GoAlone") {
        pred = follow_to_ga;
      } else if (trans.first == "Follow" && trans.second == "Pass") {
        pred = follow_to_pass;
      } else if (trans.first == "GoAlone" && trans.second == "Pass") {
        pred = ga_to_pass;
      } else if (trans.first == "GoAlone" && trans.second == "Follow") {
        pred = ga_to_follow;
      } else if (trans.first == "GoAlone" && trans.second == "Halt") {
        pred = ga_to_halt;
      } else if (trans.first == "Pass" && trans.second == "Halt") {
        pred = pass_to_halt;
      } else if (trans.first == "Pass" && trans.second == "GoAlone") {
        pred = pass_to_ga;
      } else if (trans.first == "Pass" && trans.second == "Follow") {
        pred = pass_to_follow;
      }
      // cout << pred << endl;
      if (InterpretBool(pred, example)) {
        // cout << "True" << endl;
        return trans.second;
      }
      // cout << "False" << endl;
    }
  }
  return last_state_;
}

void LoadTransitionList(const string& path) {
  std::ifstream infile(path);
  string start, output;
  while(infile >> start >> output) {
    const std::pair<string, string> trans(start, output);
    trans_list_.push_back(trans);
  }
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
  const json demo = social_lib::DemoFromRequest(req, state_);
  const Example example = MakeDemo(demo);
  // This check is to guarantee running default behavior if no humans
  // are visible.
  if (req.human_poses.size() > 0) {
    // Transition based on the demo
    state_ = Transition(example);
  } else {
    state_ = "GoAlone";
    if (state_ != last_state_) {
      cout << "Action: " << "No Humans -> Using Default Nav" << endl;
    }
  }
  if (state_ != last_state_) {
    cout << "Action: " << state_ << endl;
  }
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

  // Load the Predicates for the behavior
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
  cout << ga_to_follow << endl;
  cout << ga_to_halt << endl;
  cout << follow_to_ga << endl;
  cout << follow_to_halt << endl;
  cout << halt_to_follow << endl;
  cout << halt_to_ga << endl;

  // Initialize ROS.
  ros::init(argc, argv, "social_interp");
  ros::NodeHandle n;

  ros::ServiceServer utmrsActionRequest =
      n.advertiseService("SocialPipsSrv", ActionRequestCb);

  ros::spin();
  return 0;
}
