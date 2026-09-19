#include <ros1_comm_lab/message_processor.h>

#include <limits>

namespace ros1_comm_lab
{

MessageProcessor::MessageProcessor(
    uint32_t multiplier,
    uint32_t bias,
    uint32_t max_input)
    : multiplier_(multiplier)
    , bias_(bias)
    , max_input_(max_input)
{
}

bool MessageProcessor::process(uint32_t input, uint32_t& output) const
{
    // 先检查业务允许的输入范围；失败时直接返回，保证 output 不被覆盖。
    if (input > max_input_)
    {
        return false;
    }

    // 先提升到 uint64_t 再计算，避免 uint32_t 在乘加过程中先发生静默回绕。
    const uint64_t value =
        static_cast<uint64_t>(input) * multiplier_ + bias_;

    // 结果超出 uint32_t 可表示范围时拒绝，并继续保持 output 原值。
    if (value > std::numeric_limits<uint32_t>::max())
    {
        return false;
    }

    output = static_cast<uint32_t>(value);
    return true;
}

}  // namespace ros1_comm_lab
