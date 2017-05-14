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
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <cassert>
#include <cstdlib>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <math.h>
#include <std_msgs/String.h>
#include <unistd.h>

#include "victoria_navigation/kmeans_for_cone.h"

KmeansForCone::KmeansForCone() :
	kmeans_for_cone_server_(nh_, 
							"behavior_compute_kmeans_for_cone",
							boost::bind(&KmeansForCone::actionGoalCb, this, _1),
							false)
{
		kmeans_for_cone_server_.start();
}

void KmeansForCone::actionGoalCb(const victoria_navigation::KmeansForConeGoalConstPtr &goal) {
	victoria_navigation::KmeansForConeResult result;
	victoria_navigation::KmeansForConeFeedback feedback;
	cone_detector_topic_name_ = "/cone_detector";
	image_topic_name_ = goal->image_topic_name;
	count_object_detector_msgs_received_ = 0;
	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &KmeansForCone::coneDetectorCb, this);

	result.cone_found = false;

    actionlib::SimpleActionClient<victoria_perception::KmeansAction> ac("/compute_kmeans", true);
    feedback.feedback = "{\"progress\":\"Waiting on /compute_kmeans server\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);
    ac.waitForServer(); //will wait for infinite time

    // Send a goal to the action.
    feedback.feedback = "{\"progress\":\"Sending goal to compute kmeans action server\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);
    victoria_perception::KmeansGoal compute_kmeans_goal;
    compute_kmeans_goal.attempts = 1;
    compute_kmeans_goal.image_topic_name = image_topic_name_;
    compute_kmeans_goal.number_clusters = NUMBER_OF_CLUSTERS_;
    compute_kmeans_goal.resize_width = 320;
    ac.sendGoal(compute_kmeans_goal, NULL, NULL, boost::bind(&KmeansForCone::feedbackCb, this, _1));

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(8.0));

    if (finished_before_timeout) {
    	// Do nothing here, pick up below.
    } else if (kmeans_for_cone_server_.isPreemptRequested()) {
    	ROS_INFO("[KmeansForCone::actionGoalCb] Action was canceled");
    	kmeans_for_cone_server_.setPreempted();
    	result.cone_found = false;
		ac.cancelGoal();
		return;
    } else {
    	ROS_INFO("[KmeansForCone::actionGoalCb] TIMEOUT");
		result.cone_found = false;
		kmeans_for_cone_server_.setSucceeded(result);
    	return;
    }

    // Pick out any k-means clusters that might be a cone and merge them together.
	ClusterStatistics new_parameters = findNewConeDetectorParameters(ac.getResult()->kmeans_result);
	if (!new_parameters.valid_statistics) {
		ROS_INFO("[KmeansForCone::actionGoalCb] No good candidate for cone detector found");
		feedback.feedback = "{\"progress\":\"No good candidate for cone detector found\"}";
	    kmeans_for_cone_server_.publishFeedback(feedback);
		result.cone_found = false;
		kmeans_for_cone_server_.setSucceeded(result);
    	return;
	}

	// Set new parameters for the cone-detector and see if a cone is seen.
	result.alow_hue = new_parameters.min_hue;
	result.ahigh_hue = new_parameters.max_hue;
	result.alow_saturation = new_parameters.min_saturation;
	result.ahigh_saturation = new_parameters.max_saturation;
	result.alow_value = new_parameters.min_value;
	result.ahigh_value = new_parameters.max_value;
	ROS_INFO("[KmeansForCone::actionGoalCb] kmeans action succeeded, trying new cone_detector");
    feedback.feedback = "{\"progress\":\"Recieved result from compute kmeans action server, trying new cone_detector\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);

    // Inject the new cone_detector parameters.
    setNewConeDetectorParams(new_parameters);

    // See if the cone is detected.
    unsigned int start_cone_detected_count = 0;

	start_cone_detected_count = cone_detected_count_;
    recovery_start_sequence_number_ = count_object_detector_msgs_received_;
    while ((recovery_start_sequence_number_ + 12) > count_object_detector_msgs_received_) {
    	// The detector is considered to be working if it often detects a cone in a short period.
    	if (cone_detected_count_ > (start_cone_detected_count + 3)) {
			// Cone detector adjustments worked. Success.
			ROS_INFO("[KmeansForCone::actionGoalCb] FOUND CONE");
	    	result.cone_found = true;
	    	kmeans_for_cone_server_.setSucceeded(result);
	    	return;
	    }
    }

    // The cone-detector failed with the k-means parameters, so try again with  a first bit of
    // adjustment derived from experience.
    new_parameters.max_value += 50;
    new_parameters.max_saturation += 50;
    setSafeConeDetectorParameters(new_parameters);
	result.alow_hue = new_parameters.min_hue;
	result.ahigh_hue = new_parameters.max_hue;
	result.alow_saturation = new_parameters.min_saturation;
	result.ahigh_saturation = new_parameters.max_saturation;
	result.alow_value = new_parameters.min_value;
	result.ahigh_value = new_parameters.max_value;

	ROS_INFO("[KmeansForCone::actionGoalCb] kmeans values were not good enough, trying cone_detector with first kind of adjustments");
    feedback.feedback = "{\"progress\":\"Trying first set of adjusted parameters for cone_detector\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);
    setNewConeDetectorParams(new_parameters);
	start_cone_detected_count = cone_detected_count_;
    recovery_start_sequence_number_ = count_object_detector_msgs_received_;
    while ((recovery_start_sequence_number_ + 12) > count_object_detector_msgs_received_) {
    	// The detector is considered to be working if it often detects a cone in a short period.
    	if (cone_detected_count_ > (start_cone_detected_count + 3)) {
			// Cone detector adjustments worked. Success.
			ROS_INFO("[KmeansForCone::actionGoalCb] FOUND CONE");
	    	result.cone_found = true;
	    	kmeans_for_cone_server_.setSucceeded(result);
	    	return;
	    }
    }

    // Detector failed, try a second bit of adjustment derived from experience.
    new_parameters.min_value -= 30;
    new_parameters.min_saturation -= 50;
    setSafeConeDetectorParameters(new_parameters);
	result.alow_hue = new_parameters.min_hue;
	result.ahigh_hue = new_parameters.max_hue;
	result.alow_saturation = new_parameters.min_saturation;
	result.ahigh_saturation = new_parameters.max_saturation;
	result.alow_value = new_parameters.min_value;
	result.ahigh_value = new_parameters.max_value;
	ROS_INFO("[KmeansForCone::actionGoalCb] kmeans values were not good enough, trying cone_detector with second kind of adjustments");
    feedback.feedback = "{\"progress\":\"Trying second set of adjusted parameters for cone_detector\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);
    setNewConeDetectorParams(new_parameters);
	start_cone_detected_count = cone_detected_count_;
    recovery_start_sequence_number_ = count_object_detector_msgs_received_;
    while ((recovery_start_sequence_number_ + 12) > count_object_detector_msgs_received_) {
    	// The detector is considered to be working if it often detects a cone in a short period.
    	if (cone_detected_count_ > (start_cone_detected_count + 3)) {
			// Cone detector adjustments worked. Success.
			ROS_INFO("[KmeansForCone::actionGoalCb] FOUND CONE");
	    	result.cone_found = true;
	    	kmeans_for_cone_server_.setSucceeded(result);
	    	return;
	    }
    }

    // New cone_detector failed.
	ROS_INFO("[KmeansForCone::actionGoalCb] FAILED to find cone");
	result.cone_found = false;
	kmeans_for_cone_server_.setSucceeded(result);
	return;
}

void KmeansForCone::setNewConeDetectorParams(const ClusterStatistics& new_parameters) {
	if (!new_parameters.valid_statistics) {
		// Unsuccessful.
		ROS_INFO("[KmeansForCone::setNewConeDetectorParams] failed");
	} else {
		ClusterStatistics old_parameters = getCurrentAFilter();
		ROS_INFO("[KmeansForCone::setNewConeDetectorParams] previous A-filter min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d",
				 old_parameters.min_hue, 
				 old_parameters.max_hue,
				 old_parameters.min_saturation,
				 old_parameters.max_saturation,
				 old_parameters.min_value,
				 old_parameters.max_value);
		setCurrentAFilter(new_parameters);
		ROS_INFO("[KmeansForCone::setNewConeDetectorParams] new A-filter min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d",
				 new_parameters.min_hue, 
				 new_parameters.max_hue,
				 new_parameters.min_saturation,
				 new_parameters.max_saturation,
				 new_parameters.min_value,
				 new_parameters.max_value);
	}
}

KmeansForCone::ClusterStatistics KmeansForCone::findNewConeDetectorParameters(const std::string& kmeans_result) {
	nlohmann::json json_result = nlohmann::json::parse(kmeans_result);
	std::vector<std::string> clusters;
	std::vector<ClusterStatistics> cluster_list;
	for (nlohmann::json::iterator cluster_ptr = json_result.begin(); cluster_ptr != json_result.end(); ++cluster_ptr) {
		ClusterStatistics cluster_statistics;
		if (!parseClusterStatistics(*cluster_ptr, cluster_statistics)) {
			const std::string tmp = cluster_ptr->dump();
			ROS_ERROR("Unable to parse kmeans result for cluster istring: %s", tmp.c_str());
			return ClusterStatistics();
		} else {
			cluster_list.push_back(cluster_statistics);
		}
	}

	ClusterStatistics new_parameters = computeLikelyConeParameters(cluster_list);

	if (!new_parameters.valid_statistics) {
		// Failed to find a good set.
		return new_parameters;
	}

	setSafeConeDetectorParameters(new_parameters);
	ROS_INFO("[KmeansForCone::findNewConeDetectorParameters] new parameters:, min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d, pixel_count: %d",
			 new_parameters.min_hue, 
			 new_parameters.max_hue, 
			 new_parameters.min_saturation, 
			 new_parameters.max_saturation, 
			 new_parameters.min_value, 
			 new_parameters.max_value, 
			 new_parameters.pixels);
	return new_parameters;
}

void KmeansForCone::feedbackCb(const victoria_perception::KmeansFeedbackConstPtr& feedback) {
    //ROS_INFO("[KmeansForCone::feedbackCb] Feedback from compute kmeans action: %s", feedback->step.c_str());
}

// Capture the latest ConeDetector information
void KmeansForCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detector_msg_ = *msg;
	if (last_object_detector_msg_.object_detected) cone_detected_count_++;
	count_object_detector_msgs_received_++;
}

KmeansForCone::ClusterStatistics KmeansForCone::getCurrentAFilter() {
	ClusterStatistics result;
    assert(ros::param::get("/cone_detector/alow_hue_range", result.min_hue));
    assert(ros::param::get("/cone_detector/ahigh_hue_range", result.max_hue));
    assert(ros::param::get("/cone_detector/alow_saturation_range", result.min_saturation));
    assert(ros::param::get("/cone_detector/ahigh_saturation_range", result.max_saturation));
    assert(ros::param::get("/cone_detector/alow_value_range", result.min_value));
    assert(ros::param::get("/cone_detector/ahigh_value_range", result.max_value));
    return result;
}

void KmeansForCone::setCurrentAFilter(const KmeansForCone::ClusterStatistics& values) {
    dynamic_reconfigure::ReconfigureRequest srv_req;
    dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::IntParameter int_param;
	dynamic_reconfigure::Config conf;

	int_param.name="alow_hue_";
	int_param.value = values.min_hue;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_hue_";
	int_param.value = values.max_hue;
	conf.ints.push_back(int_param);

	int_param.name="alow_saturation_";
	int_param.value = values.min_saturation;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_saturation_";
	int_param.value = values.max_saturation;
	conf.ints.push_back(int_param);

	int_param.name="alow_value_";
	int_param.value = values.min_value;
	conf.ints.push_back(int_param);

	int_param.name="ahigh_value_";
	int_param.value = values.max_value;
	conf.ints.push_back(int_param);

	srv_req.config = conf;
	bool call_result = ros::service::call("/cone_detector/set_parameters", srv_req, srv_resp);
	if (!call_result) {
		ROS_ERROR("[KmeansForCone::setCurrentAFilter] ros::service::call failed");
	}
}

bool KmeansForCone::parseClusterStatistics(const nlohmann::json& cluster_statistics_json, KmeansForCone::ClusterStatistics& result) {
	ClusterStatistics cluster_statistics;
	std::vector<std::string> fields;

	cluster_statistics.cluster_number = cluster_statistics_json["cluster"];
	cluster_statistics.min_hue = cluster_statistics_json["min_hue"];
	cluster_statistics.max_hue = cluster_statistics_json["max_hue"];
	cluster_statistics.min_saturation = cluster_statistics_json["min_saturation"];
	cluster_statistics.max_saturation = cluster_statistics_json["max_saturation"];
	cluster_statistics.min_value = cluster_statistics_json["min_value"];
	cluster_statistics.max_value = cluster_statistics_json["max_value"];
	cluster_statistics.pixels = cluster_statistics_json["pixels"];
	// ROS_INFO("cluster: %d, min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d, pixel_count: %d",
	// 		 cluster_statistics.cluster_number, 
	// 		 cluster_statistics.min_hue, 
	// 		 cluster_statistics.max_hue, 
	// 		 cluster_statistics.min_saturation, 
	// 		 cluster_statistics.max_saturation, 
	// 		 cluster_statistics.min_value, 
	// 		 cluster_statistics.max_value, 
	// 		 cluster_statistics.pixels);

	result = cluster_statistics;
	return true;
}

bool KmeansForCone::isLikelyConeCluster(KmeansForCone::ClusterStatistics& cluster) {
	return (cluster.min_hue <= 10) &&
		   (cluster.max_hue >= 5) &&
		   (cluster.max_hue <= 40) &&
		   (cluster.pixels >= 200);
}

KmeansForCone::ClusterStatistics KmeansForCone::computeLikelyConeParameters(const std::vector<ClusterStatistics>& clusters) {
	ClusterStatistics merged_statistics;
	merged_statistics.min_hue = 179;
	merged_statistics.max_hue = 0;
	merged_statistics.min_saturation = 255;
	merged_statistics.max_saturation = 0;
	merged_statistics.min_value = 255;
	merged_statistics.max_value = 0;
	merged_statistics.pixels = 0;
	merged_statistics.valid_statistics = false;
	for (ClusterStatistics cluster : clusters) {
		if (isLikelyConeCluster(cluster)) {
			ROS_INFO("[KmeansForCone::computeLikelyConeParameters] selecting cluster: %d", cluster.cluster_number);
			merged_statistics.min_hue 		 = std::min(merged_statistics.min_hue, cluster.min_hue);
			merged_statistics.max_hue 		 = std::max(merged_statistics.max_hue, cluster.max_hue);
			merged_statistics.min_saturation = std::min(merged_statistics.min_saturation, cluster.min_saturation);
			merged_statistics.max_saturation = std::max(merged_statistics.max_saturation, cluster.max_saturation);
			merged_statistics.min_value		 = std::min(merged_statistics.min_value, cluster.min_value);
			merged_statistics.max_value		 = std::max(merged_statistics.max_value, cluster.max_value);
			merged_statistics.pixels		+= cluster.pixels;
			merged_statistics.valid_statistics = true;
		}
	}

	return merged_statistics;
}

void KmeansForCone::setSafeConeDetectorParameters(KmeansForCone::ClusterStatistics& int_out_parameters) {
	if (int_out_parameters.min_hue < 0) int_out_parameters.min_hue = 0;
	if (int_out_parameters.max_hue > 179) int_out_parameters.max_hue = 179;
	if (int_out_parameters.min_saturation < 0) int_out_parameters.min_saturation = 0;
	if (int_out_parameters.max_saturation > 255) int_out_parameters.max_saturation = 255;
	if (int_out_parameters.min_value < 0) int_out_parameters.min_value = 0;
	if (int_out_parameters.max_value > 255) int_out_parameters.max_value = 255;
}

