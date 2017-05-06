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

#ifndef __VICTORIA_NAVIGATION_KMEANS_FOR_CONE
#define __VICTORIA_NAVIGATION_KMEANS_FOR_CONE

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/bind.hpp>
#include <nav_msgs/Odometry.h>
#include <string>

#include "victoria_navigation/KmeansForConeAction.h"
#include "victoria_perception/KmeansAction.h"
#include "victoria_perception/ObjectDetector.h"

// A behavior that attempts to discover a RoboMagellan cone in the camera.
//
// The behavior depends on a few other components and parameters..
//		* "cone_detector_topic_name" is a topic listened to for an indication if a RoboMagellan cone is detected.
//		* "cmd_vel_topic_name" defines a topic to be used for moving the robot. Messages will be published
//		  to that topic. The robot will end up in a stopped state at the end of this behavior.
//		* "odometry_topic_name" defines a topic the MIGHT be listened to in order to determine the current
//		  heading. See the discussion of "imu_topic_name" above.
//
// The behavior works as follows:
//	* Wait until messages are received from the cone detector and Odometry.
//  * If the cone is seen, indicate SUCCESS and stop the robot.
//	* The first time this behavior is attempted, capture the current Odometry. This will be used
//	  to detect when a complete revolution has been made.
//	* If the robot hasn't yet made a complete revolution, rotate a bit.
//
// POSSIBLE IMPROVEMENTS:
//	* Find all cones in a complete rotation and choose the best.
//	* Choose the cone that is closest to the expected heading.

class KmeansForCone {
private:
	static const unsigned int NUMBER_OF_CLUSTERS_ = 16;

	typedef struct ClusterStatistics {
			int cluster_number = -1;
			int min_hue;
			int max_hue;
			int min_saturation;
			int max_saturation;
			int min_value;
			int max_value;
			int pixels;
			ClusterStatistics() :
				cluster_number(-1),
				min_hue(-1),
				max_hue(-1),
				min_saturation(-1),
				max_saturation(-1),
				min_value(-1),
				max_value(-1),
				pixels(-1) {}
	} ClusterStatistics;

	typedef actionlib::SimpleActionServer<victoria_navigation::KmeansForConeAction> KmeansForConeActionServer;

	// ROS handles.
	ros::NodeHandle nh_;
	KmeansForConeActionServer kmeans_for_cone_server_;

	// Parameters.
	std::string cone_detector_topic_name_;	// Topic name containing ConeDetector message.
	std::string image_topic_name_;			// Topic name containing video stream.

	// Subscribers.
	ros::Subscriber	cone_detector_sub_;

	// Algorithm variables.
	bool cone_detected_sticky_;					// Sticky bit for cone detector object_detected.
	int recovery_retry_count_;					// Recovery attempts performed.
	long int recovery_start_sequence_number_;	// Snapshot of count_object_detector_msgs_received_ for recovery.

	/*! \brief Handle the action request to perform a kmeans analysis of the
	 * current video stream and update the cone detector accordingly, if a
	 * possible cone was found in the video stream.
	 * \param goal The goal request.
	 * Return true iff the cone detector, after adjustment, succeedd in
	 * finding the cone.
	 */
	bool actionGoalCb(const victoria_navigation::KmeansForConeGoalConstPtr &goal);

	/*! \brief attempt to adjust the cone detector parameters to find a cone in the image.
	 * \param kmeans_result Set of cluster results from kmeans filter.
	 */
	void setNewConeDetectorParams(const std::string& kmeans_result);

	/*! \brief Computer a set of likely-good cone detector parameters for the current video stream. */
	ClusterStatistics findNewConeDetectorParameters(const std::string& kmeans_result); 

	ros::ServiceClient computeKmeansService_;	// For recalculating kmeans.
	bool parseClusterStatistics(const std::string& cluster_statistics_string, ClusterStatistics& result);
	bool isLikelyConeCluster(ClusterStatistics& cluster);
	ClusterStatistics computeLikelyConeParameters(const std::vector<ClusterStatistics>& clusters);

	/*! \brief Get the current A-filter values from the cone detector. */
	ClusterStatistics getCurrentAFilter();
	
	/*! \brief Put new values for the A-filter. */
	void setCurrentAFilter(ClusterStatistics& values);

	// Process one ConeDetector topic message.
	long int count_object_detector_msgs_received_;
	victoria_perception::ObjectDetector last_object_detector_msg_;
	void coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg);

	/*! \brief Make sure the cone detector parameters are valid values. */
	void setSafeConeDetectorParameters(ClusterStatistics& int_out_parameters);

	/*! \brief Process feedback from the compute kmeans action.
	 * \param feedback The feedback from the compute kmeans action.
	 */
	void feedbackCb(const victoria_perception::KmeansFeedbackConstPtr& feedback);

	// Singleton pattern.
	KmeansForCone(KmeansForCone const&) :
		kmeans_for_cone_server_(nh_, 
						"behavior_compute_kmeans_for_cone",
						boost::bind(&KmeansForCone::actionGoalCb, this, _1), false)
		{}
	KmeansForCone& operator=(KmeansForCone const&) {}

public:
	KmeansForCone();
};

#endif // __VICTORIA_NAVIGATION_KMEANS_FOR_CONE