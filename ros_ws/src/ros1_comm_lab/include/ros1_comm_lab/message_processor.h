#pragma once

#include <cstdint>

namespace ros1_comm_lab
{

/**
 * @brief 与 ROS 运行时无关的整数变换核心，用于演示“业务逻辑与 ROS I/O 解耦后更容易单测”。
 */
class MessageProcessor
{
public:
    /**
     * @brief 配置一个固定的线性变换：output = input * multiplier + bias。
     *
     * @param multiplier 输入值的乘数。
     * @param bias 乘法完成后追加的偏移量。
     * @param max_input 允许接受的最大输入值。
     */
    MessageProcessor(uint32_t multiplier, uint32_t bias, uint32_t max_input);

    /**
     * @brief 校验并转换一个输入；失败时保持 output 原值不变。
     *
     * @param input 待校验和转换的输入值。
     * @param output 成功时写入转换结果；失败时不修改。
     * @return 输入合法且计算结果未溢出 uint32_t 时返回 true，否则返回 false。
     */
    bool process(uint32_t input, uint32_t& output) const;

private:
    // 这三个成员只保存构造时的配置，process() 本身不维护可变状态，因此便于并发读取和单元测试。
    uint32_t multiplier_;
    uint32_t bias_;
    uint32_t max_input_;
};

}  // namespace ros1_comm_lab
