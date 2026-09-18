#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <std_msgs/UInt32.h>

#include <boost/bind/bind.hpp>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <sstream>
#include <string>
#include <thread>

namespace
{

std::string currentThreadId()
{
    std::ostringstream stream;
    stream << std::this_thread::get_id();
    return stream.str();
}

class SpinnerLab
{
public:
    SpinnerLab(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
        , slow_call_id_(0)
        , fast_call_id_(0)
    {
        int queue_size = 100;
        pnh_.param("queue_size", queue_size, 100);
        queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));

        pnh_.param("slow_delay", slow_delay_, 2.0);
        slow_delay_ = std::max(0.0, slow_delay_);

        pnh_.param("allow_concurrent_callbacks", allow_concurrent_callbacks_, false);
        pnh_.param<std::string>("slow_topic", slow_topic_, "/comm_lab/slow");
        pnh_.param<std::string>("fast_topic", fast_topic_, "/comm_lab/fast");

        ros::SubscribeOptions slow_options;
        slow_options.init<std_msgs::UInt32>(
            slow_topic_,
            queue_size_,
            boost::bind(&SpinnerLab::slowCallback, this, boost::placeholders::_1));
        slow_options.allow_concurrent_callbacks = allow_concurrent_callbacks_;

        slow_subscriber_ = nh_.subscribe(slow_options);
        fast_subscriber_ = nh_.subscribe(
            fast_topic_, queue_size_, &SpinnerLab::fastCallback, this);

        ROS_INFO_STREAM(
            "spinner_lab ready"
            << " slow_topic=" << slow_topic_
            << " fast_topic=" << fast_topic_
            << " queue_size=" << queue_size_
            << " slow_delay=" << slow_delay_
            << " allow_concurrent_callbacks="
            << (allow_concurrent_callbacks_ ? "true" : "false"));
    }

private:
    void slowCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        const uint64_t call_id = ++slow_call_id_;
        const std::string thread_id = currentThreadId();

        ROS_INFO_STREAM(
            "[slow] START call=" << call_id
            << " data=" << msg->data
            << " thread=" << thread_id);

        ros::WallDuration(slow_delay_).sleep();

        ROS_INFO_STREAM(
            "[slow] END   call=" << call_id
            << " data=" << msg->data
            << " thread=" << thread_id);
    }

    void fastCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        const uint64_t call_id = ++fast_call_id_;

        ROS_INFO_STREAM(
            "[fast] EVENT call=" << call_id
            << " data=" << msg->data
            << " thread=" << currentThreadId());
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber slow_subscriber_;
    ros::Subscriber fast_subscriber_;

    uint32_t queue_size_;
    double slow_delay_;
    bool allow_concurrent_callbacks_;
    std::string slow_topic_;
    std::string fast_topic_;

    std::atomic<uint64_t> slow_call_id_;
    std::atomic<uint64_t> fast_call_id_;
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "spinner_lab");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string spinner_mode;
    pnh.param<std::string>("spinner_mode", spinner_mode, "spin");

    int spinner_threads = 2;
    pnh.param("spinner_threads", spinner_threads, 2);
    spinner_threads = std::max(1, spinner_threads);

    double spin_once_period = 0.01;
    pnh.param("spin_once_period", spin_once_period, 0.01);
    spin_once_period = std::max(0.0, spin_once_period);

    SpinnerLab lab(nh, pnh);

    ROS_INFO_STREAM(
        "spinner_mode=" << spinner_mode
        << " spinner_threads=" << spinner_threads
        << " main_thread=" << currentThreadId());

    if (spinner_mode == "spin")
    {
        ros::spin();
    }
    else if (spinner_mode == "single")
    {
        ros::SingleThreadedSpinner spinner;
        spinner.spin();
    }
    else if (spinner_mode == "multi")
    {
        ros::MultiThreadedSpinner spinner(
            static_cast<uint32_t>(spinner_threads));
        spinner.spin();
    }
    else if (spinner_mode == "async")
    {
        ros::AsyncSpinner spinner(
            static_cast<uint32_t>(spinner_threads));
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    else if (spinner_mode == "spin_once")
    {
        while (ros::ok())
        {
            ros::spinOnce();
            ros::WallDuration(spin_once_period).sleep();
        }
    }
    else
    {
        ROS_ERROR_STREAM(
            "unsupported spinner_mode='" << spinner_mode
            << "'; expected spin, single, multi, async, or spin_once");
        return 2;
    }

    return 0;
}
