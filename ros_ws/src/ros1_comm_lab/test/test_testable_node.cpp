#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/UInt32.h>

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace
{

class TestableNodeIntegrationTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        output_received_ = false;
        output_value_ = 0;

        publisher_ = nh_.advertise<std_msgs::UInt32>("/comm_lab/test_input", 10);
        subscriber_ = nh_.subscribe(
            "/comm_lab/test_output",
            10,
            &TestableNodeIntegrationTest::outputCallback,
            this);
        spinner_.start();

        const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(2.0);
        while (ros::ok() && ros::WallTime::now() < deadline)
        {
            if (publisher_.getNumSubscribers() > 0 && subscriber_.getNumPublishers() > 0)
            {
                return;
            }
            ros::WallDuration(0.01).sleep();
        }

        FAIL() << "testable_node did not connect to integration test topics within 2 seconds";
    }

    void TearDown() override
    {
        spinner_.stop();
        subscriber_.shutdown();
        publisher_.shutdown();
    }

    void publishInput(uint32_t value)
    {
        std_msgs::UInt32 msg;
        msg.data = value;
        publisher_.publish(msg);
    }

    bool waitForOutput(uint32_t& value, double timeout_seconds)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool ready = condition_.wait_for(
            lock,
            std::chrono::duration<double>(timeout_seconds),
            [this] { return output_received_; });

        if (ready)
        {
            value = output_value_;
            output_received_ = false;
        }
        return ready;
    }

    void outputCallback(const std_msgs::UInt32::ConstPtr& msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            output_value_ = msg->data;
            output_received_ = true;
        }
        condition_.notify_one();
    }

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    ros::Subscriber subscriber_;
    ros::AsyncSpinner spinner_{1};
    std::mutex mutex_;
    std::condition_variable condition_;
    bool output_received_ = false;
    uint32_t output_value_ = 0;
};

TEST_F(TestableNodeIntegrationTest, PublishesTransformedNominalAndBoundaryValues)
{
    uint32_t output = 0;

    publishInput(7);
    ASSERT_TRUE(waitForOutput(output, 1.0));
    EXPECT_EQ(17u, output);

    publishInput(100);
    ASSERT_TRUE(waitForOutput(output, 1.0));
    EXPECT_EQ(203u, output);
}

TEST_F(TestableNodeIntegrationTest, DoesNotPublishForOutOfRangeInput)
{
    publishInput(101);

    uint32_t output = 0;
    EXPECT_FALSE(waitForOutput(output, 0.5));
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_testable_node");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
