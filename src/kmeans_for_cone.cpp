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

bool KmeansForCone::actionGoalCb(const victoria_navigation::KmeansForConeGoalConstPtr &goal) {
	victoria_navigation::KmeansForConeResult result;
	victoria_navigation::KmeansForConeFeedback feedback;
	cone_detector_topic_name_ = goal->cone_detector_topic_name;
	image_topic_name_ = goal->image_topic_name;
	cone_detected_sticky_ = false;
	count_object_detector_msgs_received_ = 0;
	cone_detector_sub_ = nh_.subscribe(cone_detector_topic_name_, 1, &KmeansForCone::coneDetectorCb, this);

	result.success = false;
	result.cone_detector_params = "";

    actionlib::SimpleActionClient<victoria_perception::KmeansAction> ac(goal->compute_kmeans_action_name, true);
    feedback.feedback = "{\"progress\":\"Waiting on behavior_compute_kmeans_for_cone server\"}";
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
    	ROS_INFO("[KmeansForCone::actionGoalCb] state: %s", ac.getState().toString().c_str());
    	ROS_INFO("[KmeansForCone::actionGoalCb] result msg: %s", ac.getResult()->result_msg.c_str());
    	ROS_INFO("[KmeansForCone::actionGoalCb] kmeans result: %s", ac.getResult()->kmeans_result.c_str());
    }

    if (!finished_before_timeout) {
    	ROS_INFO("[KmeansForCone::actionGoalCb] aborted");
    	kmeans_for_cone_server_.setAborted();
    	return false;
    } else if (kmeans_for_cone_server_.isPreemptRequested()) {
    	ROS_INFO("[KmeansForCone::actionGoalCb] preempted");
    	kmeans_for_cone_server_.setPreempted();
    	return false;
    }

	result.cone_detector_params = "Some Params here";//#####
	ROS_INFO("[KmeansForCone::actionGoalCb] kmeans action succeeded, trying new cone_detector");
    feedback.feedback = "{\"progress\":\"Recieved result from compute kmeans action server, trying new cone_detector\"}";
    kmeans_for_cone_server_.publishFeedback(feedback);

    // Inject the new cone_detector parameters.
    setNewConeDetectorParams(ac.getResult()->kmeans_result);

    // See if the cone is detected.
    recovery_start_sequence_number_ = count_object_detector_msgs_received_;
    while ((recovery_start_sequence_number_ + 12) > count_object_detector_msgs_received_) {
    	if (cone_detected_sticky_) {
			// Cone detector adjustments worked. Success.
			ROS_INFO("[KmeansForCone::actionGoalCb] found cone, SUCCESS");
	    	result.success = true;
	    	kmeans_for_cone_server_.setSucceeded(result);
	    	return true;
	    }
    }

    // New cone_detector failed.
	ROS_INFO("[KmeansForCone::actionGoalCb] Failed to find cone, FAILURE");
	result.success = false;
	kmeans_for_cone_server_.setAborted();
	return false;
}

void KmeansForCone::setNewConeDetectorParams(const std::string& kmeans_result) {
	ClusterStatistics new_parameters = findNewConeDetectorParameters(kmeans_result);
	if (new_parameters.max_hue == 149) {
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
	ROS_INFO("[KmeansForCone::findNewConeDetectorParameters] json_result is_array: %s", (json_result.is_array() ? "TRUE" : "FALSE"));

	std::vector<std::string> clusters;
	std::vector<ClusterStatistics> cluster_list;
	for (nlohmann::json::iterator cluster_ptr = json_result.begin(); cluster_ptr != json_result.end(); ++cluster_ptr) {
		const std::string tmp = cluster_ptr->dump();
		ROS_INFO("[KmeansForCone::findNewConeDetectorParameters] cluster string: %s", tmp.c_str());
		ClusterStatistics cluster_statistics;
		if (!parseClusterStatistics(*cluster_ptr, cluster_statistics)) {
			ROS_ERROR("Unable to parse kmeans result for cluster istring: %s", tmp.c_str());
			return ClusterStatistics();
		} else {
			cluster_list.push_back(cluster_statistics);
		}
	}

	ClusterStatistics new_parameters = computeLikelyConeParameters(cluster_list);

	if (new_parameters.min_hue == 179) {
		// ### Test for now good kmeans results.
		return new_parameters;
	}

	// Fiddle with results.
	// new_parameters.min_hue -= 2;
	// new_parameters.max_hue += 2;
	// new_parameters.min_saturation -= 4;
	// new_parameters.max_saturation += 30;
	// new_parameters.min_value -= 10;
	// new_parameters.max_value = 255;
	// setSafeConeDetectorParameters(new_parameters);
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
    ROS_INFO("[KmeansForCone::feedbackCb] Feedback from compute kmeans action: %s", feedback->step.c_str());
}

// Capture the latest ConeDetector information
void KmeansForCone::coneDetectorCb(const victoria_perception::ObjectDetectorConstPtr& msg) {
	last_object_detector_msg_ = *msg;
	cone_detected_sticky_ |= last_object_detector_msg_.object_detected;
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

void KmeansForCone::setCurrentAFilter(KmeansForCone::ClusterStatistics& values) {
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
	ROS_INFO("cluster: %d, min_hue: %d, max_hue: %d, min_saturation: %d, max_saturation: %d, min_value: %d, max_value: %d, pixel_count: %d",
			 cluster_statistics.cluster_number, 
			 cluster_statistics.min_hue, 
			 cluster_statistics.max_hue, 
			 cluster_statistics.min_saturation, 
			 cluster_statistics.max_saturation, 
			 cluster_statistics.min_value, 
			 cluster_statistics.max_value, 
			 cluster_statistics.pixels);

	result = cluster_statistics;
	return true;
}

bool KmeansForCone::isLikelyConeCluster(KmeansForCone::ClusterStatistics& cluster) {
	return (cluster.min_hue <= 7) &&
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

