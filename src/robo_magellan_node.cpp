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

#include <cassert>
#include <ros/ros.h>
#include <ros/console.h>
#include <actionlib_msgs/GoalStatus.h>
#include <actionlib/client/simple_action_client.h>

#include "victoria_navigation/discover_cone.h"
#include "victoria_navigation/KmeansForConeAction.h"
#include "victoria_navigation/move_from_cone.h"
#include "victoria_navigation/move_to_cone.h"
#include "victoria_navigation/PushGoal.h"
#include "victoria_navigation/seek_to_gps.h"
#include "victoria_navigation/solve_robomagellan.h"
#include "victoria_navigation/strategy_fn.h"
#include "victoria_perception/AnnotateDetectorImage.h"
#include <stdint.h>
#include <vector>
#include <yaml-cpp/yaml.h>

const std::vector<SolveRoboMagellan::GPS_POINT>* gps_points;
std::vector<std::string> service_goal_stack;
std::vector<int> service_point_stack;
ros::ServiceServer push_goal_service;

bool pushGoalCb(victoria_navigation::PushGoal::Request &request,
                victoria_navigation::PushGoal::Response &response) {
    if (request.push_move_from_cone) {
        service_goal_stack.push_back(MoveFromCone::singleton().goalName());
    }

    if (request.push_move_to_cone) {
        service_goal_stack.push_back(MoveToCone::singleton().goalName());
    }

    if (request.push_discover_cone) {
        service_goal_stack.push_back(DiscoverCone::singleton().goalName());
    }

    if (request.push_seek_to_gps) {
        service_goal_stack.push_back(SeekToGps::singleton().goalName());
        service_point_stack.push_back(request.point_number_to_seek_to);
        if ((service_point_stack.back() < 0) || (service_point_stack.back() >= gps_points->size())) {
            std::ostringstream ss;
            ss << "point_number_to_seek must be in [0.." << gps_points->size() << "]";
            response.result = ss.str();
            return false;
        }
    }

    if (request.push_solve_robomagellan) {
        service_goal_stack.push_back(SolveRoboMagellan::singleton().goalName());
    }

    if (request.execute_now) {
        while (!service_goal_stack.empty()) {
            std::string goal_name = service_goal_stack.back();
            if (goal_name == SeekToGps::singleton().goalName()) {
                StrategyFn::pushGoal(goal_name, "");
                StrategyFn::pushGpsPoint(gps_points->at(service_point_stack.back()));
                ROS_INFO_NAMED("robo_magellan_node", "[pushGoalCb] pushing goal: %s, point: %d", goal_name.c_str(), service_point_stack.back());
                service_point_stack.pop_back();
            } else {
                StrategyFn::pushGoal(goal_name, "");
                ROS_INFO_NAMED("robo_magellan_node", "[pushGoalCb] pushing goal: %s", goal_name.c_str());
            }

            service_goal_stack.pop_back();
        }

        // The following clear() calls should be redundant, stacks should have been cleared above.
        service_goal_stack.clear();
        service_point_stack.clear();
    } else {
        for (std::string goal_name : service_goal_stack) {
            ROS_INFO_NAMED("robo_magellan_node", "[pushGoalCb] enqueuing goal: %s", goal_name.c_str());
        }
    }

    return true;
}

// Perform problem solving by iterating over each of the possible problem solvers, a.k.a.
// behaviors. Each solver is asked if it can solve the current problem. The response is normally
// either:
//      *   INACTIVE    => The behavior isn't applicable to the current problem.
//      *   SUCCESS     => Yes, the behavior solved the problem.
//      *   RUNNING     => The behavior is working on solving the problem..
//      *   FATAL       => Something unexpected happened and wasn't handled. Abort everything.
//
// When a behavior works on a problem, it can subdivide the problem into sub problems by pushing
// a new set of problems onto the stack of outstanding problems. When a behavior solves the current
// problem (i.e., the top of the stasck problem), it is responsible for removing that problem
// from the stack. When the last problem is solved (i.e., the stack is empty), this node exits.
//
// Input:
//      behaviors                   A list of possible behaviors.
//      strategyStatusPublisher     A ROS publisher that reports internal strategy progress or information.
//
    void doStrategy(std::vector<StrategyFn*>& behaviors, ros::Publisher& strategyStatusPublisher, ros::ServiceClient annotatorService) {
    actionlib_msgs::GoalStatus goalStatus;
    StrategyFn::RESULT_T result = StrategyFn::FATAL;
    ros::Rate rate(30); // Loop rate

    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();

        for (std::vector<StrategyFn*>::iterator it = behaviors.begin(); it != behaviors.end(); ++it) {
            // Ask one of the behaviors to try to solve the current problem.
            StrategyFn::RESULT_T result = ((*it)->tick)();

            // Generate a report on what just happened, showing what behavior was invoked and its response.
            goalStatus.goal_id.stamp = ros::Time::now();
            goalStatus.goal_id.id = "robo_magellan_node";
            goalStatus.status = actionlib_msgs::GoalStatus::ACTIVE;
            goalStatus.text = "[strategy_node] executed: " 
                              + ((*it)->name()) 
                              + ", result: " + StrategyFn::resultToString(result)
                              + ", currentGoalName: " + StrategyFn::currentGoalName().c_str();
            strategyStatusPublisher.publish(goalStatus);

            if (result == StrategyFn::INACTIVE) {
                // The behavior wasn't applicable to the goal. 
                // Advance to the next behavior to see if it can solve the problem.
                continue;
            }

            if (result == StrategyFn::FATAL) {
                // Something bad happened -- kill the problem solver.
                ROS_INFO_STREAM("[_strategy_node] function: " << ((*it)->name()) << ", FATAL result, exiting");
                return;
            } else if (result == StrategyFn::RUNNING) {
                // The behavior is working on a solution. Advance to the next behavior which
                // might also want a crack at solving the problem.
                continue;
            } else if (result == StrategyFn::SUCCESS) {
                // The current problem was solved by the behavior.
                if (StrategyFn::goalStackEmpty()) {
                    // There are no more problems to be solved, we're done.
                    continue;
                } else {
                    // There are still other problems to be solved. Start again.
                    break;
                }
            } else if (result == StrategyFn::FAILED) {
                // The current behavior tried by failed to solve the problem.
                // Just carry on, giving the problem to the other behaviors.
                continue;
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robo_magellan_node");

    // ROS node handle.
    ros::NodeHandle nh("~");

    actionlib::SimpleActionClient<victoria_navigation::KmeansForConeAction> ac("behavior_compute_kmeans_for_cone", true);
    ROS_INFO("[robo_magellan_node] Waiting for KmeansForCone action server to start.");
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("[robo_magellan_node] KmeansForCone action server has started");

    push_goal_service = nh.advertiseService("push_goal", &pushGoalCb);
    ros::Publisher strategyStatusPublisher = nh.advertise<actionlib_msgs::GoalStatus>("/strategy", 1);

    // A list of all possible problem solvers/subgoals/behaviors.
    std::vector<StrategyFn*> behaviors;

    // Add all possible behaviors to the list.
    behaviors.push_back(&DiscoverCone::singleton());
    behaviors.push_back(&MoveFromCone::singleton());
    behaviors.push_back(&MoveToCone::singleton());
    behaviors.push_back(&SeekToGps::singleton());
    behaviors.push_back(&SolveRoboMagellan::singleton());

    gps_points = ((SolveRoboMagellan&) SolveRoboMagellan::singleton()).getGpsPoints();

    ros::ServiceClient coneDetectorAnnotatorService = nh.serviceClient<victoria_perception::AnnotateDetectorImage>("/cone_detector/annotate_detector_image", true);

    // Set the initial problem to be solved.
    //StrategyFn::pushGoal(SolveRoboMagellan::singleton().goalName(), "0");

    // Attempt to solve the problem.
    doStrategy(behaviors, strategyStatusPublisher, coneDetectorAnnotatorService);

    return 0;
}

