// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "navigation/NavClient.h"

namespace navigation
{
    NavClient::NavClient()
    : ac_("move_base", true)
    {
        ROS_INFO("Waiting for action server to start.");
		ac_.waitForServer();
		ROS_INFO("Action server started, sending goal.");
    }

    void
    NavClient::doWork(move_base_msgs::MoveBaseGoal goal)
    {
        ROS_INFO("Sending action");

		ac_.sendGoal(goal_,
			boost::bind(&NavClient::doneCb, this, _1, _2),
			Client::SimpleActiveCallback(),
			boost::bind(&NavClient::feedbackCb, this, _1));
		
        ROS_INFO("Action sent");
    }

    void 
    NavClient::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
	{
		ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
	}

	void 
    NavClient::doneCb(const actionlib::SimpleClientGoalState& state, const move_base_msgs::MoveBaseResultConstPtr& result)
	{
		ROS_INFO("Finished in state [%s]", state.toString().c_str());
		ros::shutdown();
	}

}  // namespace navigation