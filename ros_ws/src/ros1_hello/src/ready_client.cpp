#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <string>

namespace
{
void chatterCallback(const std_msgs::String::ConstPtr& message)
{
    ROS_INFO("[ready_client] business callback: %s", message->data.c_str());
}
}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ready_client");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    double wait_timeout = 10.0;
    pnh.param("wait_timeout", wait_timeout, 10.0);

    const std::string ready_service_name = "/demo_driver/ready";
    ROS_INFO("[ready_client] waiting for %s", ready_service_name.c_str());

    // 依赖的是 READY 条件，不依赖 launch 文件中的 <node> 书写顺序，也不使用固定 sleep 猜启动时间。
    if (!ros::service::waitForService(ready_service_name, ros::Duration(wait_timeout)))
    {
        ROS_ERROR("[ready_client] timeout waiting for %s", ready_service_name.c_str());
        return 1;
    }

    ros::ServiceClient ready_client = nh.serviceClient<std_srvs::Trigger>(ready_service_name);
    std_srvs::Trigger ready_request;
    if (!ready_client.call(ready_request) || !ready_request.response.success)
    {
        ROS_ERROR("[ready_client] readiness check failed");
        return 1;
    }

    ROS_INFO("[ready_client] dependency ready: %s", ready_request.response.message.c_str());

    // 只有依赖确认 READY 后才进入业务路径，开始订阅前面章节已有的 /chatter。
    ros::Subscriber chatter_sub = nh.subscribe("/chatter", 10, chatterCallback);
    ROS_INFO("[ready_client] business path enabled; subscribed to /chatter");

    ros::spin();
    return 0;
}
