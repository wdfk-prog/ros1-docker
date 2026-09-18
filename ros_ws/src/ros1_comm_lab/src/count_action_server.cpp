#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <ros1_comm_lab/CountAction.h>

#include <boost/bind/bind.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace
{

class CountActionServer
{
public:
    using Server = actionlib::SimpleActionServer<ros1_comm_lab::CountAction>;

    CountActionServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
        , valid_(false)
        , step_period_(0.1)
    {
        pnh_.param<std::string>("action_name", action_name_, "/comm_lab/count");
        pnh_.param("step_period", step_period_, 0.1);
        if (step_period_ < 0.0)
        {
            ROS_FATAL_STREAM("~step_period must be non-negative, got " << step_period_);
            return;
        }

        server_ = std::make_unique<Server>(
            nh_,
            action_name_,
            boost::bind(&CountActionServer::execute, this, boost::placeholders::_1),
            false);
        server_->start();
        valid_ = true;

        ROS_INFO_STREAM(
            "count_action_server ready"
            << " action_name=" << action_name_
            << " step_period=" << step_period_);
    }

    bool valid() const
    {
        return valid_;
    }

private:
    bool finishIfPreempted(uint32_t completed_count)
    {
        if (!server_->isPreemptRequested())
        {
            return false;
        }

        ros1_comm_lab::CountResult result;
        result.completed = false;
        result.final_count = completed_count;
        result.message = "goal preempted";
        server_->setPreempted(result, result.message);
        return true;
    }

    void execute(const ros1_comm_lab::CountGoalConstPtr& goal)
    {
        uint32_t completed_count = 0;

        while (completed_count < goal->target)
        {
            if (!ros::ok() || finishIfPreempted(completed_count))
            {
                return;
            }

            ros::WallDuration(step_period_).sleep();

            // Cancel can arrive while the simulated work step blocks, so re-check before committing progress.
            if (!ros::ok() || finishIfPreempted(completed_count))
            {
                return;
            }

            ++completed_count;

            ros1_comm_lab::CountFeedback feedback;
            feedback.current_count = completed_count;
            feedback.progress =
                100.0F * static_cast<float>(completed_count)
                / static_cast<float>(goal->target);
            server_->publishFeedback(feedback);
        }

        // A cancel/new goal can arrive after the last loop check but before the terminal state is committed.
        if (!ros::ok() || finishIfPreempted(completed_count))
        {
            return;
        }

        ros1_comm_lab::CountResult result;
        result.completed = true;
        result.final_count = completed_count;
        result.message = "goal completed";
        server_->setSucceeded(result, result.message);
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    std::unique_ptr<Server> server_;
    std::string action_name_;
    bool valid_;
    double step_period_;
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_action_server");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    CountActionServer server(nh, pnh);

    if (!server.valid())
    {
        return 2;
    }

    ros::spin();
    return 0;
}
