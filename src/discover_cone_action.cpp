// Copyright 2017 Michael Wimble

// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.

// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.

// 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse
// or promote products derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
// BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <angles/angles.h>
#include <cstdlib>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include "victoria_navigation/discover_cone_action.h"

DiscoverConeAction::DiscoverConeAction() :
	discover_cone_server_(nh_,
						  "behavior_discover_cone",
						  boost::bind(&DiscoverConeAction::actionGoalCb, this, _1),
						  false)
{
	assert(ros::param::get("~cmd_vel_topic_name", cmd_vel_topic_name_));
	assert(ros::param::get("~image_topic_name", image_topic_name_));
	assert(ros::param::get("~odometry_topic_name", odometry_topic_name_));
	assert(ros::param::get("~yaw_turn_radians_per_sec", yaw_turn_radians_per_sec_));

	ROS_DEBUG_NAMED("discover_cone_action", "[DiscoverConeAction] PARAM cmd_vel_topic_name: %s", cmd_vel_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone_action", "[DiscoverConeAction] PARAM image_topic_name: %s", image_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone_action", "[DiscoverConeAction] PARAM odometry_topic_name: %s", odometry_topic_name_.c_str());
	ROS_DEBUG_NAMED("discover_cone_action", "[DiscoverConeAction] PARAM yaw_turn_radians_per_sec: %7.4f", yaw_turn_radians_per_sec_);

	discover_cone_server_.start();
}

void DiscoverConeAction::actionGoalCb(const victoria_navigation::DiscoverConeGoalConstPtr &goal) {
	victoria_navigation::DiscoverConeResult result;		// For sending the action result.
	victoria_navigation::DiscoverConeFeedback feedback;	// For sending the action feedback.

	// Listen to odometry messages which are authorative about the robot's pose.
	odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &DiscoverConeAction::odometryCb, this);

	// Publish to this topic to rotate the robot.
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name_.c_str(), 1);
	
	// Begin by assuming the action will fail.
	result.discovered_cone = false;

	// For using the "behavior_compute_kmeans_for_cone" action to attempt to find a cone in the video stream.
    actionlib::SimpleActionClient<victoria_navigation::KmeansForConeAction> ac("/behavior_compute_kmeans_for_cone", true);
 
    feedback.feedback = "{\"progress\":\"Waiting on behavior_compute_kmeans_for_cone server\"}";
    discover_cone_server_.publishFeedback(feedback);
    ac.waitForServer(); // Wait for the "behavior_compute_kmeans_for_cone" action server to come online.

    // Wait until odometry messsages start to arrive.
    ros::Rate rate(5); // Spin 5 times/second
	while (ros::ok() && 
		  (count_odometry_msgs_received_ <= 0) &&
		  !discover_cone_server_.isPreemptRequested()) {
	    feedback.feedback = "{\"progress\":\"Waiting on odometry messages\"}";
	    discover_cone_server_.publishFeedback(feedback);
	    rate.sleep();
	}

	doCaptureState(); // Capture interesting state at the start of the action.
	recovery_retry_count_ = 0;	// Counts recovery attempts.

    victoria_navigation::KmeansForConeGoal kmeans_for_cone_goal; // For invoking the "behavior_compute_kmeans_for_cone" action.
    kmeans_for_cone_goal.image_topic_name = image_topic_name_;

	while (true) {
		// Attempt to adjust the cone detector in various ways to see if a cone can be
		// detected in the video stream.
	    ac.sendGoal(kmeans_for_cone_goal, NULL, NULL, boost::bind(&DiscoverConeAction::feedbackCb, this, _1));
	    bool finished_before_timeout = ac.waitForResult(ros::Duration(12.0));

	    if (finished_before_timeout) {
	    	if (ac.getResult()->cone_found) {
	    		// Cone found.
			    feedback.feedback = "{\"progress\":\"SUCCESS Found cone\"}";
			    discover_cone_server_.publishFeedback(feedback);
	    		ROS_INFO("[DiscoverConeAction::actionGoalCb] SUCCESS");
	    		result.discovered_cone = true;
	    		discover_cone_server_.setSucceeded(result);
	    		break;
	    	} else {
	    		// Failed to discover cone in current position. Rotate.
				computeTotalRotatedYaw();
	    		if (total_rotated_yaw_ <= (2 * M_PI)) {
	    			// Haven't completed one total rotaton yet, rotate some more.
				    feedback.feedback = "{\"progress\":\"NO CONE DETECTED, rotating\"}";
				    discover_cone_server_.publishFeedback(feedback);

					geometry_msgs::Twist	cmd_vel;		// For sending movement commands to the robot.

					cmd_vel.linear.x = 0;
					cmd_vel.angular.z = yaw_turn_radians_per_sec_ / 2.0;
					cmd_vel_pub_.publish(cmd_vel);
	    		} else {
	    			// Rotated at least a complete circle without discovering a cone in the video stream.
					if (recovery_retry_count_ > ALLOWED_CIRCLE_ROTATE_RETRIES) {
						// Done with retries.
					    feedback.feedback = "{\"progress\":\"FAILED to find cone after all roations\"}";
					    discover_cone_server_.publishFeedback(feedback);
			    		result.discovered_cone = false;
			    		discover_cone_server_.setSucceeded(result);
			    		break;
					} else {
						// More retries are available.
					    feedback.feedback = "{\"progress\":\"FAILED to find cone after one rotation, retrying\"}";
					    discover_cone_server_.publishFeedback(feedback);
						recovery_retry_count_++;

				    	// Reset data for another rotation
				    	doCaptureState();
					}
	    		}
	    	}
	    } else if (discover_cone_server_.isPreemptRequested()) {
	    	ROS_INFO("[DiscoverConeAction::actionGoalCb] PREEMPTED");
		    feedback.feedback = "{\"progress\":\"Action was canceled\"}";
		    discover_cone_server_.publishFeedback(feedback);
	    	discover_cone_server_.setPreempted();
			result.discovered_cone = false;
			ac.cancelGoal(); // Cancel upstream actions.
	    	break;
	    } else {
	    	// Timed out.
	    	ROS_INFO("[DiscoverConeAction::actionGoalCb] TIMEOUT");
		    feedback.feedback = "{\"progress\":\"Action not completed in time\"}";
		    discover_cone_server_.publishFeedback(feedback);
			result.discovered_cone = false;
			discover_cone_server_.setSucceeded(result);
			break;
	    }
	}
}

// Capture the lates Odometry information.
void DiscoverConeAction::odometryCb(const nav_msgs::OdometryConstPtr& msg) {
	last_odometry_msg_ = *msg;
	count_odometry_msgs_received_++;	
}

void DiscoverConeAction::doCaptureState() {
	starting_odometry_msg_ = last_odometry_msg_;
	previous_pose_ = last_odometry_msg_.pose.pose.orientation;
	starting_yaw_ = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
	total_rotated_yaw_ = 0;
}

double DiscoverConeAction::computeTotalRotatedYaw() {
	tf::Quaternion 				current_tf_quat;
	double 						current_yaw = 0.0;
	double 						original_yaw = 0.0;
	tf::Quaternion 				previous_tf_quat;

	tf::quaternionMsgToTF(previous_pose_, previous_tf_quat);
	tf::quaternionMsgToTF(last_odometry_msg_.pose.pose.orientation, current_tf_quat);
	original_yaw = tf::getYaw(starting_odometry_msg_.pose.pose.orientation);
	current_yaw = tf::getYaw(last_odometry_msg_.pose.pose.orientation);
	total_rotated_yaw_ = total_rotated_yaw_ + fabs(previous_tf_quat.angleShortestPath(current_tf_quat));
	previous_pose_ = last_odometry_msg_.pose.pose.orientation;
}

void DiscoverConeAction::feedbackCb(const victoria_navigation::KmeansForConeFeedbackConstPtr& feedback) {
    ROS_INFO("[DiscoverConeAction::feedbackCb] Feedback from KmeansForConeAction action: %s", feedback->feedback.c_str());
}

