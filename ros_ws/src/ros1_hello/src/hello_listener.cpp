#include <ros/ros.h>
#include <std_msgs/String.h>

// 每收到一条 /chatter 消息，ros::spin() 会让 roscpp 调用这个函数。
static void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("received: %s", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hello_listener");

    ros::NodeHandle nh;

    // 订阅 /chatter，接收队列长度为 10。
    ros::Subscriber subscriber = nh.subscribe("chatter", 10, chatterCallback);

    // 持续处理 callback queue，直到 Ctrl+C 或 ros::shutdown()。
    ros::spin();

    return 0;
}
