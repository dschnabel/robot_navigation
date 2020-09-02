#include <dwb_critics/recovery.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core2/exceptions.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::RecoveryCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

void RecoveryCritic::onInit()
{
    reset();
}

void RecoveryCritic::reset()
{
    stuck_counter_ = 0;
    stuck_ = false;
    chosen_x_ = 0.0;
}

bool RecoveryCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
        const geometry_msgs::Pose2D& goal,
        const nav_2d_msgs::Path2D& global_plan)
{
    double vel_x = fabs(vel.x);
    if (!stuck_) {
        if (vel_x <= 0.02 && chosen_x_ > 0.1) {
            stuck_counter_++;
            if (stuck_counter_ > 3) {
                stuck_counter_ = 0;
                stuck_ = true;
                stuck_pose_ = pose;
                ROS_INFO_NAMED("RecoveryCritic", "Robot seems stuck. Starting recovery...");
            }
        } else {
            stuck_counter_ = 0;
        }
    } else {
        if (vel_x <= 0.02 && chosen_x_ > 0.1) {
            stuck_counter_++;
            if (stuck_counter_ > 25) {
                throw nav_core2::OccupiedStartException("Cannot unstuck robot.");
            }
        } else {
            stuck_counter_ = 0;
        }

        double dx = pose.x - stuck_pose_.x;
        double dy = pose.y - stuck_pose_.y;
        double sq_dist = dx * dx + dy * dy;
        if (sq_dist > 0.50 * 0.50) {
            stuck_ = false;
            ROS_INFO_NAMED("RecoveryCritic", "Recovery finished.");
        }
    }

    return true;
}

double RecoveryCritic::getScale() const
{
    // we're not stuck, don't worry about recovery
    if (!stuck_) {
        return 0.0;
    }

    return scale_;
}

double RecoveryCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
    if (traj.velocity.x >= 0 || traj.velocity.x < -0.15 || traj.velocity.x > -0.10) {
        return 100.0;
    }
    return fabs(traj.velocity.theta);
}

void RecoveryCritic::debrief(const nav_2d_msgs::Twist2D& cmd_vel)
{
    chosen_x_ = fabs(cmd_vel.x);
}

} /* namespace dwb_critics */
