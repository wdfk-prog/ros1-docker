#include <ros/ros.h>
#include <ros1_hello/HelloStatus.h>

#include <cstdint>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "custom_msg_publisher");

    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<ros1_hello::HelloStatus>("hello_status", 10);

    ros::Rate rate(1.0);
    int32_t sequence = 0;

    while (ros::ok())
    {
        ros1_hello::HelloStatus msg;
        msg.sequence = sequence++;
        msg.text = "hello from custom .msg";

        ROS_INFO_STREAM("publish: sequence=" << msg.sequence << " text=" << msg.text);
        publisher.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
