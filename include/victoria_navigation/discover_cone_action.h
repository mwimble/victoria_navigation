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

#ifndef __VICTORIA_NAVIGATION_DISCOVER_CONE_ACTION
#define __VICTORIA_NAVIGATION_DISCOVER_CONE_ACTION

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include "victoria_navigation/discover_cone_action.h"
#include "victoria_navigation/DiscoverConeAction.h"
#include "victoria_navigation/KmeansForConeAction.h"
#include "victoria_perception/ObjectDetector.h"

// An action that attempts to discover a RoboMagellan trafic cone in the camera.
// The action works as follows:
// * Wait for the "behavior_compute_kmeans_for_cone" action server to come online.
// * Wait for odometry messages to show up.
// * Loop
// *	Invoke the "behavior_compute_kmeans_for_cone" action to attempt to adjust the cone detector 
//		to find any cone in in the video stream.
// *	If the cone detector reports a cone is found, this action SUCCEEDS and terminates.
// *	If a complete rotation of the robot hasn't been done, rotate a bit and start the loop again.
// *	If more rotations are allowed, setup to start another rotation and start the loop again.
// *	Otherwise the action FAILES -- it was unable to find the cone after looking around and
//		reajusting the cone detector.
//
// The following configuration parameters are required.
// * cmd_vel_topic_name 		For sending rotate commands.
// * image_topic_name 			Passed on to downstream actions for reading the video stream.
// * odometry_topic_name 		For tracking the rotation of the robot.
// * yaw_turn_radians_per_sec 	Value sent for each request to rotate the robot a bit.

class DiscoverConeAction {
private:
	const unsigned int ALLOWED_CIRCLE_ROTATE_RETRIES = 1; // Allowed to rotate once, and then once more.

	typedef actionlib::SimpleActionServer<victoria_navigation::DiscoverConeAction> DiscoverConeActionServer;

	// ROS handles.
	ros::NodeHandle nh_;					// ROS node handle.
	ros::Publisher cmd_vel_pub_;			// For sending rotate commands.
	DiscoverConeActionServer discover_cone_server_;	// Action to handle the dynamic cone detector.
	ros::Subscriber odometry_sub_;			// For listening to odometry messsages.

	// Parameters.
	std::string cmd_vel_topic_name_;		// Topic name containing cmd_vel message.
	std::string image_topic_name_;			// Topic name containing video stream.
	std::string odometry_topic_name_;		// Topic name containing Odometry message.
	float yaw_turn_radians_per_sec_;		// Rate to turn around z azis (radians/sec)

	// Handle for accepting and acting upon this action's goal.
	void actionGoalCb(const victoria_navigation::DiscoverConeGoalConstPtr &goal);

	// Algorithm variables.
	geometry_msgs::Quaternion previous_pose_;	// Pose from last Odometry message.
	int recovery_retry_count_;					// Recovery attempts performed.
	nav_msgs::Odometry starting_odometry_msg_;	// Odometry mesage at start of rotation strategy.
	double starting_yaw_;						// Starting yaw.
	double total_rotated_yaw_;					// Integration of rotational yaw since start.

	/*! \brief update captured state of total rotated yaw. */
	double computeTotalRotatedYaw();

	/*! \brief Start of problem solver. Capture interesting state. */
	void doCaptureState();

	// Process one Odometry topic message;
	long int count_odometry_msgs_received_;
	nav_msgs::Odometry last_odometry_msg_;
	void odometryCb(const nav_msgs::OdometryConstPtr& msg);

	/*! \brief Process feedback from the compute kmeans action.
	 * \param feedback The feedback from the compute kmeans action.
	 */
	void feedbackCb(const victoria_navigation::KmeansForConeFeedbackConstPtr& feedback);

	DiscoverConeAction(DiscoverConeAction const&) :
		discover_cone_server_(nh_,
						  "behavior_discover_cone",
						  boost::bind(&DiscoverConeAction::actionGoalCb, this, _1),
						  false)
	    {};
	DiscoverConeAction& operator=(DiscoverConeAction const&) {};

public:
	DiscoverConeAction();
};

#endif // __VICTORIA_NAVIGATION_DISCOVER_CONE_ACTION