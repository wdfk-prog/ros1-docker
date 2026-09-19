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

// 日志打印线程 ID，用来直观看不同 Spinner 模式是否让 callback 并发执行。
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

        // SubscribeOptions 比普通 nh.subscribe() 暴露更多订阅配置。
        // 这里专门用于演示同一个 Subscriber 是否允许多个 callback 并发进入。
        ros::SubscribeOptions slow_options;
        slow_options.init<std_msgs::UInt32>(
            slow_topic_,
            queue_size_,
            boost::bind(&SpinnerLab::slowCallback, this, boost::placeholders::_1));
        // false 时，同一 Subscriber 的消息 callback 保持串行；true 时允许并发执行。
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
    // 慢回调用 WallDuration 模拟耗时工作，便于观察它是否阻塞其它 callback。
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

    // 快回调不 sleep；如果它仍明显延迟，说明当前 Spinner/队列模型发生了串行阻塞。
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

    // spinner_mode 用同一份 Node 切换多种 callback 消费模型，便于比较行为。
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
        // 最常见写法：当前线程单线程消费默认 CallbackQueue。
        ros::spin();
    }
    else if (spinner_mode == "single")
    {
        ros::SingleThreadedSpinner spinner;
        spinner.spin();
    }
    else if (spinner_mode == "multi")
    {
        // MultiThreadedSpinner 阻塞当前线程，同时用多个 worker 消费默认队列。
        ros::MultiThreadedSpinner spinner(
            static_cast<uint32_t>(spinner_threads));
        spinner.spin();
    }
    else if (spinner_mode == "async")
    {
        // AsyncSpinner 在后台线程消费 callback；当前线程可以继续做其它工作或等待 shutdown。
        ros::AsyncSpinner spinner(
            static_cast<uint32_t>(spinner_threads));
        spinner.start();
        ros::waitForShutdown();
        spinner.stop();
    }
    else if (spinner_mode == "spin_once")
    {
        // spinOnce() 适合“主循环自己掌控节奏”的程序；每轮主动处理一次当前可用 callback。
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
