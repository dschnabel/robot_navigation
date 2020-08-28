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

#include <dwb_critics/prefer_forward.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::PreferForwardCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

void PreferForwardCritic::onInit()
{
  critic_nh_.param("backward_cost", backwardCost_, 1.0);
  critic_nh_.param("forward_cost", forwardCost_, 0.0);
}

bool PreferForwardCritic::prepare(const geometry_msgs::Pose2D& pose,
        const nav_2d_msgs::Twist2D& vel,
        const geometry_msgs::Pose2D& goal,
        const nav_2d_msgs::Path2D& global_plan)
{
    if (!startRotationReached_) {
        if (startTheta_ == 0.0) {
            int index = global_plan.poses.size() < 10 ? global_plan.poses.size() : 10;
            auto &p1 = global_plan.poses[0];
            auto &p2 = global_plan.poses[index];
            startTheta_ = atan2(p2.y-p1.y, p2.x-p1.x);
        }
        double diff = startTheta_ - pose.theta;
        while (diff < -M_PI) diff += M_PI*2;
        while (diff > M_PI) diff -= M_PI*2;
        if (fabs(diff) < 0.2) {
            startRotationReached_ = true;
        }
    }
    return true;
}

double PreferForwardCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  // backward motions bad on a robot without backward sensors
  if (traj.velocity.x < 0.0) {
    return backwardCost_;
  } else if (traj.velocity.x > 0.0 && !startRotationReached_) {
      return backwardCost_;
  } else {
      return forwardCost_;
  }
}

void PreferForwardCritic::reset()
{
    startRotationReached_ = false;
    startTheta_ = 0.0;
}

} /* namespace dwb_critics */
