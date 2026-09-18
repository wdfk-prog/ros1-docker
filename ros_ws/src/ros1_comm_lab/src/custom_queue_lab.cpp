#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>

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

class CustomQueueLab
{
public:
    CustomQueueLab(
        ros::NodeHandle& global_nh,
        ros::NodeHandle& driver_nh,
        ros::NodeHandle& pnh)
        : global_nh_(global_nh)
        , driver_nh_(driver_nh)
        , pnh_(pnh)
        , fast_call_id_(0)
        , driver_call_id_(0)
    {
        int queue_size = 100;
        pnh_.param("queue_size", queue_size, 100);
        queue_size_ = static_cast<uint32_t>(std::max(1, queue_size));

        pnh_.param("driver_delay", driver_delay_, 2.0);
        driver_delay_ = std::max(0.0, driver_delay_);

        pnh_.param<std::string>("fast_topic", fast_topic_, "/comm_lab/fast");
        pnh_.param<std::string>("driver_topic", driver_topic_, "/comm_lab/driver");

        fast_subscriber_ = global_nh_.subscribe(
            fast_topic_, queue_size_, &CustomQueueLab::fastCallback, this);

        driver_subscriber_ = driver_nh_.subscribe(
            driver_topic_, queue_size_, &CustomQueueLab::driverCallback, this);

        ROS_INFO_STREAM(
            "custom_queue_lab ready"
            << " fast_topic=" << fast_topic_
            << " driver_topic=" << driver_topic_
            << " queue_size=" << queue_size_
            << " driver_delay=" << driver_delay_);
    }

private:
    void fastCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        const uint64_t call_id = ++fast_call_id_;

        ROS_INFO_STREAM(
            "[global] EVENT call=" << call_id
            << " data=" << msg->data
            << " thread=" << currentThreadId());
    }

    void driverCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        const uint64_t call_id = ++driver_call_id_;
        const std::string thread_id = currentThreadId();

        ROS_INFO_STREAM(
            "[driver] START call=" << call_id
            << " data=" << msg->data
            << " thread=" << thread_id);

        ros::WallDuration(driver_delay_).sleep();

        ROS_INFO_STREAM(
            "[driver] END   call=" << call_id
            << " data=" << msg->data
            << " thread=" << thread_id);
    }

    ros::NodeHandle& global_nh_;
    ros::NodeHandle& driver_nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber fast_subscriber_;
    ros::Subscriber driver_subscriber_;

    uint32_t queue_size_;
    double driver_delay_;
    std::string fast_topic_;
    std::string driver_topic_;

    std::atomic<uint64_t> fast_call_id_;
    std::atomic<uint64_t> driver_call_id_;
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_queue_lab");

    /* Keep the custom queue alive longer than its NodeHandle, subscribers, and spinner. */
    ros::CallbackQueue driver_queue;

    ros::NodeHandle global_nh;
    ros::NodeHandle driver_nh;
    ros::NodeHandle pnh("~");

    driver_nh.setCallbackQueue(&driver_queue);

    CustomQueueLab lab(global_nh, driver_nh, pnh);

    ros::AsyncSpinner driver_spinner(1, &driver_queue);
    driver_spinner.start();

    ROS_INFO_STREAM(
        "global queue uses ros::spin() on main_thread=" << currentThreadId()
        << "; driver queue uses one AsyncSpinner worker");

    ros::spin();

    driver_spinner.stop();
    return 0;
}
