#include <ros/ros.h>
#include <std_msgs/String.h>

#include <sstream>

int main(int argc, char** argv)
{
    // 初始化 roscpp，并把默认 Node 名设置为 hello_node。
    ros::init(argc, argv, "hello_node");

    // 普通 NodeHandle 用来创建 Publisher、Subscriber 等 ROS 对象。
    ros::NodeHandle nh;

    // "~" 表示当前 Node 的 private namespace。
    ros::NodeHandle pnh("~");

    double publish_rate = 1.0;

    // 读取 ~publish_rate；如果参数不存在，就使用默认值 1.0 Hz。
    pnh.param("publish_rate", publish_rate, 1.0);

    // 创建 std_msgs/String Publisher，Topic 为 /chatter，发送队列长度为 10。
    ros::Publisher publisher = nh.advertise<std_msgs::String>("chatter", 10);

    ros::Rate rate(publish_rate);
    int count = 0;

    while (ros::ok())
    {
        std_msgs::String msg;
        std::ostringstream stream;
        stream << "hello ros1 from docker: " << count++;
        msg.data = stream.str();

        ROS_INFO("%s", msg.data.c_str());
        publisher.publish(msg);

        // 处理当前 callback queue；这个示例没有订阅回调，但保留它用于学习 API。
        ros::spinOnce();

        // 按 publish_rate 控制循环频率。
        rate.sleep();
    }

    return 0;
}
