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

// 把 std::thread::id 转成字符串，便于从日志直接观察 callback 实际在哪个线程执行。
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

        // global_nh_ 使用默认 CallbackQueue，后面由主线程 ros::spin() 处理。
        fast_subscriber_ = global_nh_.subscribe(
            fast_topic_, queue_size_, &CustomQueueLab::fastCallback, this);

        // driver_nh_ 会绑定到独立 CallbackQueue，慢回调不会堵住默认队列。
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
    // 快回调只记录事件；通过日志中的 thread 字段可与 driver 回调线程对比。
    void fastCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        const uint64_t call_id = ++fast_call_id_;

        ROS_INFO_STREAM(
            "[global] EVENT call=" << call_id
            << " data=" << msg->data
            << " thread=" << currentThreadId());
    }

    // 慢回调用 sleep 模拟阻塞式设备 I/O，故意占用 driver 专用 callback 线程。
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

    // 自定义 CallbackQueue 必须比绑定它的 NodeHandle / Subscriber / Spinner 活得更久，
    // 所以在这些对象之前创建，并在 main() 退出时最后销毁。
    ros::CallbackQueue driver_queue;

    ros::NodeHandle global_nh;
    ros::NodeHandle driver_nh;
    ros::NodeHandle pnh("~");

    // 只把 driver_nh 绑定到自定义队列；global_nh 仍使用默认全局队列。
    driver_nh.setCallbackQueue(&driver_queue);

    CustomQueueLab lab(global_nh, driver_nh, pnh);

    // 单独启动 1 个 worker 处理 driver_queue；主线程继续 ros::spin() 默认队列。
    ros::AsyncSpinner driver_spinner(1, &driver_queue);
    driver_spinner.start();

    ROS_INFO_STREAM(
        "global queue uses ros::spin() on main_thread=" << currentThreadId()
        << "; driver queue uses one AsyncSpinner worker");

    ros::spin();

    driver_spinner.stop();
    return 0;
}
