#pragma once

#include <cstdint>

namespace ros1_comm_lab
{

/**
 * @brief Apply a bounded integer transform that can be tested without ROS runtime dependencies.
 */
class MessageProcessor
{
public:
    /**
     * @brief Construct a processor with fixed transform and input limits.
     *
     * @param multiplier Value multiplied with each accepted input.
     * @param bias Value added after multiplication.
     * @param max_input Largest accepted input value.
     */
    MessageProcessor(uint32_t multiplier, uint32_t bias, uint32_t max_input);

    /**
     * @brief Transform one input while preserving the output argument on failure.
     *
     * @param input Value to validate and transform.
     * @param output Receives the transformed value when the call succeeds.
     * @return true when the input is within range and the result fits in uint32_t; false otherwise.
     */
    bool process(uint32_t input, uint32_t& output) const;

private:
    uint32_t multiplier_;
    uint32_t bias_;
    uint32_t max_input_;
};

}  // namespace ros1_comm_lab
