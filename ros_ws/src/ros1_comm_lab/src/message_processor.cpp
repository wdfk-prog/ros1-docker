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
    if (input > max_input_)
    {
        return false;
    }

    const uint64_t value =
        static_cast<uint64_t>(input) * multiplier_ + bias_;

    if (value > std::numeric_limits<uint32_t>::max())
    {
        return false;
    }

    output = static_cast<uint32_t>(value);
    return true;
}

}  // namespace ros1_comm_lab
