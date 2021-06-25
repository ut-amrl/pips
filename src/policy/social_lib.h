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
\file    social_lib.h
\author  Jarrett Holtz, (C) 2021
*/
//========================================================================
//
#include <dlfcn.h>
#include <string.h>
#include <stdlib.h>
#include <vector>

#include "amrl_msgs/NavStatusMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/Localization2DMsg.h"
#include "amrl_msgs/NavigationConfigMsg.h"
#include "amrl_msgs/SocialPipsSrv.h"
#include "amrl_msgs/HumanStateMsg.h"

#include "amrl_shared_lib/math/geometry.h"
#include "amrl_shared_lib/math/math_util.h"

#include "eigen3/Eigen/Dense"

#include <nlohmann/json.hpp>


#ifndef SOCIAL_NAVIGATION_H
#define SOCIAL_NAVIGATION_H

namespace social_lib {

Eigen::Vector2f VecFromMsg(const amrl_msgs::Pose2Df& msg);

nlohmann::json MakeEntry(const std::string& name,
               const float& value,
               const std::vector<int>& dim);

nlohmann::json MakeEntry(const std::string& name, const std::string& value);


nlohmann::json GetHumanJson();

nlohmann::json GetHumanJson(const amrl_msgs::SocialPipsSrv::Request& req);

amrl_msgs::HumanStateMsg GetClosest(
      const std::vector<amrl_msgs::HumanStateMsg>& humans);

void GetRelevantHumans(const amrl_msgs::SocialPipsSrv::Request& req,
                       amrl_msgs::HumanStateMsg* front_human,
                       amrl_msgs::HumanStateMsg* front_l,
                       amrl_msgs::HumanStateMsg* front_r);

void WriteDemos(const std::vector<nlohmann::json>& demos,
                const std::string& output_name);

float StraightFreePathLength(const Eigen::Vector2f& start,
                             const Eigen::Vector2f& end,
                             const std::vector<amrl_msgs::Pose2Df>& human_poses);

nlohmann::json DemoFromRequest(const amrl_msgs::SocialPipsSrv::Request& req,
                               const std::string& action);

nlohmann::json MakeDemo(const amrl_msgs::SocialPipsSrv::Request& req);

}

#endif  // SOCIAL_LIB_H
