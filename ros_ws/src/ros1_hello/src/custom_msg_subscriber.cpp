#include <ros/ros.h>
#include <ros1_hello/HelloStatus.h>

// 收到 hello_status 消息后，roscpp 会把只读消息指针交给这个回调函数。
// ConstPtr 避免不必要的消息拷贝，并保证回调不会修改 ROS 交付的消息对象。
static void statusCallback(const ros1_hello::HelloStatus::ConstPtr& msg)
{
    ROS_INFO_STREAM("received: sequence=" << msg->sequence << " text=" << msg->text);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_msg_subscriber");

    ros::NodeHandle nh;

    // subscribe() 把 Topic、接收队列长度和 callback 绑定起来。
    ros::Subscriber subscriber = nh.subscribe("hello_status", 10, statusCallback);

    // ros::spin() 持续处理默认 CallbackQueue，直到 Ctrl+C 或 ros::shutdown()。
    ros::spin();
    return 0;
}
