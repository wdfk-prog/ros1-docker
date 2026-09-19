#include <ros/ros.h>

#include <ros1_comm_lab/TransformValue.h>

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

// 把命令行字符串严格解析成 uint32_t；完整消费字符串且不越界才算成功。
bool parseUint32(const char* text, uint32_t& value)
{
    try
    {
        std::size_t parsed = 0;
        const unsigned long long converted = std::stoull(text, &parsed, 10);
        if (text[parsed] != '\0' || converted > std::numeric_limits<uint32_t>::max())
        {
            return false;
        }

        value = static_cast<uint32_t>(converted);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_service_client");

    if (argc != 2)
    {
        std::cerr << "usage: transform_service_client <input>" << std::endl;
        return 2;
    }

    uint32_t input = 0;
    if (!parseUint32(argv[1], input))
    {
        std::cerr << "input must be an unsigned 32-bit integer" << std::endl;
        return 2;
    }

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Service 名称做成 private 参数，便于 launch/remap 时复用同一个 client。
    std::string service_name;
    pnh.param<std::string>("service_name", service_name, "/comm_lab/transform_value");

    // 创建同步 ServiceClient；waitForExistence() 先等待 Master 中出现对应 Service。
    ros::ServiceClient client = nh.serviceClient<ros1_comm_lab::TransformValue>(service_name);
    if (!client.waitForExistence(ros::Duration(2.0)))
    {
        ROS_ERROR_STREAM("service did not become available within 2 seconds: " << service_name);
        return 3;
    }

    // .srv 会生成一个同时包含 request/response 的 C++ Service 类型。
    ros1_comm_lab::TransformValue service;
    service.request.input = input;

    // call() 为同步 RPC：返回前会等待 server 处理完成或底层调用失败。
    if (!client.call(service))
    {
        ROS_ERROR_STREAM("service call failed: " << service_name);
        return 4;
    }

    std::cout
        << "success=" << (service.response.success ? "true" : "false")
        << " output=" << service.response.output
        << " message=\"" << service.response.message << "\""
        << std::endl;

    return service.response.success ? 0 : 5;
}
