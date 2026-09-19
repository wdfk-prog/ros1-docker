#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace
{
// Trigger Service 的 server callback。返回 true 表示这次 RPC 已被正常处理；
// response.success 才是这个示例定义的“业务 READY”状态。
bool handleReady(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response)
{
    response.success = true;
    response.message = "demo_driver is ready";
    return true;
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ready_server");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // 用可配置的延时模拟真实驱动初始化，例如打开设备、读取配置、等待硬件稳定。
    double startup_delay = 3.0;
    pnh.param("startup_delay", startup_delay, 3.0);

    ROS_INFO("[ready_server] simulating driver initialization for %.1f s", startup_delay);
    if (startup_delay > 0.0)
    {
        ros::WallDuration(startup_delay).sleep();
    }

    // Service 只在“驱动初始化”完成后发布；Service 的可发现性就是本实验的 READY 条件。
    ros::ServiceServer ready_service = nh.advertiseService("/demo_driver/ready", handleReady);
    ROS_INFO("[ready_server] READY: service /demo_driver/ready is available");

    ros::spin();
    return 0;
}
