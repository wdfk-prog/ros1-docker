#include <ros/ros.h>
#include <ros1_hello/HelloStatus.h>

#include <cstdint>

int main(int argc, char** argv)
{
    // 初始化 ROS1 C++ 客户端库，并注册本 Node 的默认名称。
    ros::init(argc, argv, "custom_msg_publisher");

    // NodeHandle 是创建 Publisher / Subscriber / Service 等 ROS 对象的入口。
    ros::NodeHandle nh;

    // advertise<T>() 声明：本 Node 将在 hello_status Topic 上发布 HelloStatus 消息。
    // 第二个参数 10 是发送队列长度；短时间内来不及发送的消息最多在队列中保留 10 条。
    ros::Publisher publisher = nh.advertise<ros1_hello::HelloStatus>("hello_status", 10);

    // 1 Hz 表示主循环目标频率为每秒 1 次。
    ros::Rate rate(1.0);
    int32_t sequence = 0;

    while (ros::ok())
    {
        // HelloStatus 是本 package 的 msg/HelloStatus.msg 自动生成出的 C++ 消息类型。
        ros1_hello::HelloStatus msg;
        msg.sequence = sequence++;
        msg.text = "hello from custom .msg";

        ROS_INFO_STREAM("publish: sequence=" << msg.sequence << " text=" << msg.text);
        publisher.publish(msg);

        // 本例没有订阅回调，spinOnce() 主要用于保持与后续“循环 + callback”写法一致。
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
