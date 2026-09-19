#include <ros/ros.h>

#include <ros1_comm_lab/TransformValue.h>
#include <ros1_comm_lab/message_processor.h>

#include <cstdint>
#include <memory>
#include <string>

namespace
{

// 从 private namespace 读取非负整数参数，并统一做边界检查。
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

class TransformServiceServer
{
public:
    TransformServiceServer(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        : nh_(nh)
        , pnh_(pnh)
        , valid_(false)
        , response_delay_(0.0)
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

        pnh_.param<std::string>("service_name", service_name_, "/comm_lab/transform_value");
        pnh_.param("response_delay", response_delay_, 0.0);
        if (response_delay_ < 0.0)
        {
            ROS_FATAL_STREAM("~response_delay must be non-negative, got " << response_delay_);
            return;
        }

        // 业务计算放进独立 MessageProcessor；Service callback 只负责 ROS 请求/响应适配。
        processor_ = std::make_unique<ros1_comm_lab::MessageProcessor>(
            multiplier, bias, max_input);

        // advertiseService() 把 Service 名和成员函数 callback 注册到 ROS Master/CallbackQueue。
        service_ = nh_.advertiseService(
            service_name_,
            &TransformServiceServer::handleRequest,
            this);
        valid_ = true;

        ROS_INFO_STREAM(
            "transform_service_server ready"
            << " service_name=" << service_name_
            << " multiplier=" << multiplier
            << " bias=" << bias
            << " max_input=" << max_input
            << " response_delay=" << response_delay_);
    }

    bool valid() const
    {
        return valid_;
    }

private:
    bool handleRequest(
        ros1_comm_lab::TransformValue::Request& request,
        ros1_comm_lab::TransformValue::Response& response)
    {
        // 人为阻塞用于观察“同步 Service 会占用处理它的 callback 线程”这一行为。
        if (response_delay_ > 0.0)
        {
            ros::WallDuration(response_delay_).sleep();
        }

        uint32_t output = 0;
        if (!processor_->process(request.input, output))
        {
            // RPC 传输本身已经成功到达 server，因此 callback 仍返回 true；
            // “输入不合法”属于业务失败，通过 response.success/message 告诉 client。
            response.success = false;
            response.output = 0;
            response.message = "input violates processor constraints";
            return true;
        }

        response.success = true;
        response.output = output;
        response.message = "ok";
        return true;
    }

    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    std::unique_ptr<ros1_comm_lab::MessageProcessor> processor_;
    ros::ServiceServer service_;
    std::string service_name_;
    bool valid_;
    double response_delay_;
};

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_service_server");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    TransformServiceServer server(nh, pnh);

    if (!server.valid())
    {
        return 2;
    }

    ros::spin();
    return 0;
}
