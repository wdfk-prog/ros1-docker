#include <gtest/gtest.h>

#include <ros1_comm_lab/message_processor.h>

#include <cstdint>
#include <limits>

// 正常输入：验证 output = input * multiplier + bias 的基本路径。
TEST(MessageProcessorTest, ProcessesNominalInput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 0;
    ASSERT_TRUE(processor.process(7, output));
    EXPECT_EQ(17u, output);
}

// 边界输入：max_input 本身应该被接受。
TEST(MessageProcessorTest, AcceptsMaximumConfiguredInput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 0;
    ASSERT_TRUE(processor.process(100, output));
    EXPECT_EQ(203u, output);
}

// 非法输入：超过 max_input 时返回失败，并且不能破坏调用者原有 output。
TEST(MessageProcessorTest, RejectsInputAboveConfiguredLimitWithoutChangingOutput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 55;
    EXPECT_FALSE(processor.process(101, output));
    EXPECT_EQ(55u, output);
}

// 算术边界：即使输入合法，只要乘加结果超出 uint32_t 也必须拒绝且保持 output。
TEST(MessageProcessorTest, RejectsArithmeticOverflowWithoutChangingOutput)
{
    const ros1_comm_lab::MessageProcessor processor(
        2,
        1,
        std::numeric_limits<uint32_t>::max());

    uint32_t output = 77;
    EXPECT_FALSE(processor.process(std::numeric_limits<uint32_t>::max(), output));
    EXPECT_EQ(77u, output);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
