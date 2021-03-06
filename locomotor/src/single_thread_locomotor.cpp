/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <locomotor/locomotor.h>
#include <locomotor/locomotor_action_server.h>
#include <nav_2d_utils/conversions.h>
#include <string>
#include <thread>
#include <actionlib/client/simple_action_client.h>
#include <rovy_ros_helper.h>
#include <nav_grid/coordinate_conversion.h>

namespace locomotor
{
using nav_core2::getResultCode;

/**
 * @class SingleThreadLocomotor
 * @brief Connect the callbacks in Locomotor to do global planning once and then local planning on a timer
 *
 * When a new goal is recieved, it triggers a global costmap update
 * When that finishes, it requests a global plan.
 * When that finishes, it starts a timer to update the local costmap.
 * When the local costmap update finishes, it requests a local plan.
 * When the goal is reached, it stops the timer.
 */
class SingleThreadLocomotor
{
public:
  explicit SingleThreadLocomotor(const ros::NodeHandle& private_nh)
    : private_nh_(private_nh), locomotor_(private_nh_), main_ex_(private_nh_, false),
      as_(private_nh_, std::bind(&SingleThreadLocomotor::setGoal, this, std::placeholders::_1)),
      ac_("/locomotor/navigate", true), rovyActionId_(0)
  {
    private_nh_.param("controller_frequency", controller_frequency_, controller_frequency_);
    desired_control_duration_ = ros::Duration(1.0 / controller_frequency_);
    control_loop_timer_ = private_nh_.createTimer(desired_control_duration_,
                                                  &SingleThreadLocomotor::controlLoopCallback,
                                                  this, false, false);  // one_shot=false(default), auto_start=false
    locomotor_.initializeGlobalCostmap(main_ex_);
    locomotor_.initializeGlobalPlanners(main_ex_);
    locomotor_.initializeLocalCostmap(main_ex_);
    locomotor_.initializeLocalPlanners(main_ex_);

    tele_sub_ = private_nh_.subscribe("/key_vel", 1, &SingleThreadLocomotor::keyboardCallback, this);
    timer_ = private_nh_.createTimer(ros::Duration(0.5), &SingleThreadLocomotor::timerCallback, this, true);
    was_started_ = false;
    local_plannig_exception_count_ = 0;

    RovyRosHelper &rosHelper = RovyRosHelper::getInstance(NULL, &private_nh_);
    rosHelper.receiverRegister<rovy::GoalAction, rovy::GoalSetCallback>(
            std::bind(&SingleThreadLocomotor::goalSetCallback, this,
                    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
                    std::placeholders::_4, std::placeholders::_5)
    );
  }

  void setGoal(nav_2d_msgs::Pose2DStamped goal)
  {
    local_plannig_exception_count_ = 0;
    locomotor_.setGoal(goal);
    locomotor_.requestGlobalCostmapUpdate(main_ex_, main_ex_,
      std::bind(&SingleThreadLocomotor::onGlobalCostmapUpdate, this, std::placeholders::_1),
      std::bind(&SingleThreadLocomotor::onGlobalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
  }

protected:
  void requestGlobalCostmapUpdate()
  {
    locomotor_.requestGlobalCostmapUpdate(main_ex_, main_ex_,
      std::bind(&SingleThreadLocomotor::onGlobalCostmapUpdate, this, std::placeholders::_1),
      std::bind(&SingleThreadLocomotor::onGlobalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void requestNavigationFailure(const locomotor_msgs::ResultCode& result)
  {
    locomotor_.requestNavigationFailure(main_ex_, result,
      std::bind(&SingleThreadLocomotor::onNavigationFailure, this, std::placeholders::_1));
  }

  // Locomotor Callbacks
  void onGlobalCostmapUpdate(const ros::Duration& planning_time)
  {
    locomotor_.requestGlobalPlan(main_ex_, main_ex_,
      std::bind(&SingleThreadLocomotor::onNewGlobalPlan, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SingleThreadLocomotor::onGlobalPlanningException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void onGlobalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::GLOBAL_COSTMAP, getResultCode(e_ptr),
                                            "Global Costmap failure."));
  }

  void onNewGlobalPlan(nav_2d_msgs::Path2D new_global_plan, const ros::Duration& planning_time)
  {
    locomotor_.publishPath(new_global_plan);
    locomotor_.getCurrentLocalPlanner().setPlan(new_global_plan);
    control_loop_timer_.start();
    as_.publishFeedback(locomotor_.getNavigationState());
  }

  void onGlobalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    ROS_ERROR_NAMED("Locomotor", "Global planning error. Giving up.");
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::GLOBAL_PLANNER, getResultCode(e_ptr),
                                            "Global Planning Failure."));
  }

  void controlLoopCallback(const ros::TimerEvent& event)
  {
    locomotor_.requestLocalCostmapUpdate(main_ex_, main_ex_,
      std::bind(&SingleThreadLocomotor::onLocalCostmapUpdate, this, std::placeholders::_1),
      std::bind(&SingleThreadLocomotor::onLocalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void onLocalCostmapUpdate(const ros::Duration& planning_time)
  {
    locomotor_.requestLocalPlan(main_ex_, main_ex_,
      std::bind(&SingleThreadLocomotor::onNewLocalPlan, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SingleThreadLocomotor::onLocalPlanningException, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&SingleThreadLocomotor::onNavigationCompleted, this));
  }

  void onLocalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::LOCAL_COSTMAP, getResultCode(e_ptr),
                                            "Local Costmap failure."));
  }

  void onNewLocalPlan(nav_2d_msgs::Twist2DStamped new_command, const ros::Duration& planning_time)
  {
    locomotor_.publishTwist(new_command);
    if (planning_time > desired_control_duration_)
    {
      ROS_WARN_NAMED("locomotor", "Control loop missed its desired rate of %.4fHz... "
                     "the loop actually took %.4f seconds (>%.4f).",
                     controller_frequency_, planning_time.toSec(), desired_control_duration_.toSec());
    }
    as_.publishFeedback(locomotor_.getNavigationState());
    local_plannig_exception_count_ = 0;
  }

  void onLocalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    int code = -1;
    std::string message;
    try {
        std::rethrow_exception(e_ptr);
    } catch (const nav_core2::NavCore2Exception& e) {
        code = e.getResultCode();
        message = e.what();
    }
    if (code == 7) {
        ROS_ERROR_NAMED("Locomotor", "Fatal local planning error. Giving up.");
        requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::LOCAL_PLANNER, code, message));
    } else if (local_plannig_exception_count_++ > 10) {
        ROS_WARN_NAMED("Locomotor", "Attempt rotation for recovery.");
        locomotor_.setRecoveryMode(1);
    } else {
        ROS_WARN_NAMED("Locomotor", "Local planning error. Creating new global plan.");
        control_loop_timer_.stop();
        requestGlobalCostmapUpdate();
    }
  }

  void onNavigationCompleted()
  {
    ROS_INFO_NAMED("Locomotor", "Plan completed! Stopping.");
    control_loop_timer_.stop();

    // notify other services that navigation is complete
    publishDoneMsg(rovyActionId_, rovy::status::OK);

    nav_2d_msgs::Twist2DStamped cmd_vel;
    cmd_vel.header.stamp = ros::Time::now();
    cmd_vel.velocity.x = 0;
    cmd_vel.velocity.theta = 0;
    locomotor_.publishTwist(cmd_vel);

    as_.completeNavigation();
  }

  void onNavigationFailure(const locomotor_msgs::ResultCode result)
  {
    control_loop_timer_.stop();

    // notify other services that navigation is complete
    publishDoneMsg(rovyActionId_, rovy::status::SUBSCRIBER_ERR);

    ROS_ERROR_NAMED("Locomotor", "Failure details: %s", result.message.c_str());

    as_.failNavigation(result);
  }

  void keyboardCallback(const geometry_msgs::TwistConstPtr& msg) {
      if (control_loop_timer_.hasStarted()) {
          ROS_INFO_NAMED("Locomotor", "Manual drive activated. Stopping Loop.");
          control_loop_timer_.stop();
          was_started_ = true;
      }
      timer_.stop();
      timer_.start();
  }

  void timerCallback(const ros::TimerEvent& event) {
      if (was_started_) {
          ROS_INFO_NAMED("Locomotor", "Auto drive activated. Starting Loop.");
          was_started_ = false;
          requestGlobalCostmapUpdate();
      }
  }

  void goalSetCallback(uint16_t id, int8_t goalType, float x, float y, float theta) {
      locomotor_msgs::NavigateToPoseGoal goal;
      goal.goal.header.frame_id = "map";

      nav_2d_msgs::Pose2DStamped currentPose = locomotor_.getGlobalRobotPose();

      if (goalType == rovy::goal_type::CUSTOM) {
          goal.goal.pose.x = x;
          goal.goal.pose.y = y;
          goal.goal.pose.theta = (theta * M_PI / 180.0);
      } else if (goalType == rovy::goal_type::ROTATE_ANGLE) {
          goal.goal.pose.x = currentPose.pose.x;
          goal.goal.pose.y = currentPose.pose.y;
          goal.goal.pose.theta = currentPose.pose.theta + (theta * M_PI / 180.0);
      } else if (goalType == rovy::goal_type::ROTATE_180) {
          goal.goal.pose.x = currentPose.pose.x;
          goal.goal.pose.y = currentPose.pose.y;
          goal.goal.pose.theta = currentPose.pose.theta + (180 * M_PI / 180.0);
      } else if (goalType == rovy::goal_type::FORWARD) {
          nav_core2::Costmap::Ptr costmap = locomotor_.getLocalCostmap();
          geometry_msgs::Pose2D forward_pose;
          unsigned int cell_x, cell_y;
          unsigned char cost;
          float distance = 0.1;
          bool withinGrid;

          // get furthest possible distance we can travel in a straight line
          do {
              distance += 0.1;
              forward_pose = getForwardPose(currentPose.pose, distance);
              withinGrid = worldToGridBounded(costmap->getInfo(), forward_pose.x, forward_pose.y, cell_x, cell_y);
              cost = (*costmap)(cell_x, cell_y);
              ROS_INFO_NAMED("Locomotor", "d: %f, cell_x: %d, cell_y: %d, cost: %d, withinGrid: %d",
                      distance, cell_x, cell_y, cost, withinGrid);
          } while (withinGrid && cost != costmap->LETHAL_OBSTACLE
                  && cost != costmap->NO_INFORMATION);

          if (distance > 0.3) {
              distance -= 0.3;
              forward_pose = getForwardPose(currentPose.pose, distance);
          }

          // verify goal using global planner
          bool goodGoal = false;
          do {
              try {
                  nav_2d_msgs::Pose2DStamped p;
                  p.header = currentPose.header;
                  p.pose = forward_pose;
                  locomotor_.getCurrentGlobalPlanner().makePlan(currentPose, p);
                  goodGoal = true;
              } catch (const nav_core2::PlannerException& e) {
                  ROS_INFO_NAMED("Locomotor", "Bad goal!!! (distance: %f)", distance);
                  distance -= 0.1;
                  forward_pose = getForwardPose(currentPose.pose, distance);
              }
          } while (!goodGoal && distance > 0);

          goal.goal.pose.x = forward_pose.x;
          goal.goal.pose.y = forward_pose.y;
          goal.goal.pose.theta = forward_pose.theta;
      } else {
          ROS_INFO_NAMED("Locomotor", "Unknown goal type.");
          return;
      }

      rovyActionId_ = id;
      ac_.sendGoal(goal);
  }

  void publishDoneMsg(uint16_t id, int8_t status) {
      RovyRosHelper::getInstance().done<rovy::GoalAction>(id, status);
  }

  // distance (in meters)
  geometry_msgs::Pose2D getForwardPose(const geometry_msgs::Pose2D& pose, double distance) {
      geometry_msgs::Pose2D forward_pose;
      forward_pose.x = pose.x + distance * cos(pose.theta);
      forward_pose.y = pose.y + distance * sin(pose.theta);
      forward_pose.theta = pose.theta;
      return forward_pose;
  }

  ros::NodeHandle private_nh_;
  // Locomotor Object
  Locomotor locomotor_;

  // Timer Stuff
  double controller_frequency_ { 20.0 };
  ros::Duration desired_control_duration_;
  ros::Timer control_loop_timer_;

  // Main Executor using Global CallbackQueue
  Executor main_ex_;

  // Action Server
  LocomotorActionServer as_;

  ros::Subscriber tele_sub_;
  ros::Timer timer_;
  bool was_started_;
  int local_plannig_exception_count_;
  actionlib::SimpleActionClient<locomotor_msgs::NavigateToPoseAction> ac_;
  int rovyActionId_;
};
};  // namespace locomotor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "locomotor_node");
  ros::NodeHandle nh("~");
  locomotor::SingleThreadLocomotor sm(nh);
  ros::spin();
  return 0;
}
