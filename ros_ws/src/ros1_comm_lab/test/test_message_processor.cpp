#include <gtest/gtest.h>

#include <ros1_comm_lab/message_processor.h>

#include <cstdint>
#include <limits>

TEST(MessageProcessorTest, ProcessesNominalInput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 0;
    ASSERT_TRUE(processor.process(7, output));
    EXPECT_EQ(17u, output);
}

TEST(MessageProcessorTest, AcceptsMaximumConfiguredInput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 0;
    ASSERT_TRUE(processor.process(100, output));
    EXPECT_EQ(203u, output);
}

TEST(MessageProcessorTest, RejectsInputAboveConfiguredLimitWithoutChangingOutput)
{
    const ros1_comm_lab::MessageProcessor processor(2, 3, 100);

    uint32_t output = 55;
    EXPECT_FALSE(processor.process(101, output));
    EXPECT_EQ(55u, output);
}

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
