#include <ros/ros.h>
#include <std_msgs/UInt32.h>

#include <ros1_comm_lab/message_processor.h>

#include <cstdint>
#include <memory>
#include <string>

namespace
{

// 统一读取非负整数参数，避免构造函数里重复相同的参数校验代码。
bool readNonNegativeParam(
    ros::NodeHandle& pnh,
    const std::string& name,
    int default_value,
    uint32_t& output)
{
    int value = default_value;
    pnh.param(name, value, default_value);

    if (value < 0)
    {
        ROS_FATAL_STREAM("~" << name << " must be non-negative, got " << value);
        return false;
    }

    output = static_cast<uint32_t>(value);
    return true;
}

class TestableNode
{
public:
    TestableNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
        , valid_(false)
    {
        uint32_t multiplier = 0;
        uint32_t bias = 0;
        uint32_t max_input = 0;

        if (!readNonNegativeParam(pnh_, "multiplier", 2, multiplier)
            || !readNonNegativeParam(pnh_, "bias", 3, bias)
            || !readNonNegativeParam(pnh_, "max_input", 100, max_input))
        {
            return;
        }

        pnh_.param<std::string>("input_topic", input_topic_, "/comm_lab/test_input");
        pnh_.param<std::string>("output_topic", output_topic_, "/comm_lab/test_output");

        // 纯业务计算放在 MessageProcessor，Node 只负责参数、Topic 和日志等 ROS 适配。
        processor_ = std::make_unique<ros1_comm_lab::MessageProcessor>(
            multiplier, bias, max_input);

        publisher_ = nh_.advertise<std_msgs::UInt32>(output_topic_, 10);
        subscriber_ = nh_.subscribe(
            input_topic_, 10, &TestableNode::inputCallback, this);
        valid_ = true;

        ROS_INFO_STREAM(
            "testable_node ready"
            << " input_topic=" << input_topic_
            << " output_topic=" << output_topic_
            << " multiplier=" << multiplier
            << " bias=" << bias
            << " max_input=" << max_input);
    }

    bool valid() const
    {
        return valid_;
    }

private:
    // Topic callback 只做“ROS 消息 -> 纯业务函数 -> ROS 消息”的薄适配。
    void inputCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        uint32_t output = 0;
        if (!processor_->process(msg->data, output))
        {
            ROS_WARN_STREAM("drop input=" << msg->data << " because it violates processor constraints");
            return;
        }

        std_msgs::UInt32 output_msg;
        output_msg.data = output;
        publisher_.publish(output_msg);
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    std::unique_ptr<ros1_comm_lab::MessageProcessor> processor_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    std::string input_topic_;
    std::string output_topic_;
    bool valid_;
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testable_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TestableNode node(nh, pnh);

    if (!node.valid())
    {
        return 2;
    }

    ros::spin();
    return 0;
}
