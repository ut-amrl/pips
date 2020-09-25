#include <amrl_msgs/Pose2Df.h>
#include <eigen3/Eigen/Core>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ut_multirobot_sim/Localization2DMsg.h>
#include <gflags/gflags.h>

#include "../ast.hpp"
#include "../enumeration.hpp"
#include "../visitors/deepcopy_visitor.hpp"
#include "../visitors/fillhole_visitor.hpp"
#include "../visitors/interp_visitor.hpp"
#include "../visitors/print_visitor.hpp"

using namespace AST;
using namespace Eigen;
using namespace std;
using namespace std_msgs;
using Localization2DMsg = ut_multirobot_sim::Localization2DMsg;
using Pose2Df = amrl_msgs::Pose2Df;

DEFINE_bool(dim_checking, true, "Should dimensions be checked?");
DEFINE_bool(sig_pruning, true, "Should signature pruning be enabled?");

Model model;
bool have_data = false;

void LocalizationCallback(const Localization2DMsg& msg)
{
  model["robot_pose_x"] = make_shared<Num>(Num(msg.pose.x, {1, 0, 0}));
  model["robot_pose_y"] = make_shared<Num>(Num(msg.pose.y, {1, 0, 0}));
  model["robot"] = make_shared<Num>(Num(msg.pose.theta, {1, 0, 0}));
  have_data = true;
}

int main(int argc, char* argv[])
{
  ROS_INFO("Starting sketch runner!");

  ROS_INFO("Connecting to ROS");
  ros::init(argc, argv, "sketch_runner");
  ros::NodeHandle n;
  ros::Publisher robot_state_pub = n.advertise<String>("/robot_state", 1);
  ros::Subscriber localization_sub = n.subscribe("/localization", 1, &LocalizationCallback);

  ROS_INFO("Constructing sketch");
  const ast_ptr if_far_away =
      make_shared<BinOp>(
          make_shared<Feature>(Feature("robot_pose_x", Vector3i(1, 0, 0), NUM)),
          make_shared<Num>(Num(23.0f, {1, 0, 0})),
          "Gt"
      );
  const ast_ptr otherwise = make_shared<Bool>(Bool(true));

  const Sketch sketch = {
    make_pair(if_far_away, SymEntry(string("GO_ALONE"))),
    make_pair(otherwise, SymEntry(string("STOP")))
  };

  ros::Rate loop(20.0);
  while (ros::ok()) {
    ROS_DEBUG("Going through main loop again.");
    ros::spinOnce();

    if (have_data) {
      for (const auto& p : sketch) {
        ast_ptr new_ast = DeepCopyAST(p.first);
        FillHoles(new_ast, model);
        //cout << new_ast << endl;
        new_ast = Interpret(new_ast);
        
        if (new_ast->type_ == BOOL) {
          ROS_INFO("New AST is of type bool");
          bool_ptr b = dynamic_pointer_cast<Bool>(new_ast);

          if (b->value_) {
            ROS_INFO("New AST is true");
            const string new_command = p.second.GetString();
            ROS_INFO("PUBLISHING");
            robot_state_pub.publish(new_command);
            break;
          }
          else {
              ROS_INFO("New AST is false.");
          }
        }
        else {
          ROS_INFO("AST actually has type: %d", new_ast->type_);
        }
      }
    }
    loop.sleep();
  }

  return EXIT_SUCCESS;
}