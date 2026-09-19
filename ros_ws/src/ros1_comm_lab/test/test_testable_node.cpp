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
        // 本测试自己充当上游 Publisher 和下游 Subscriber，与被测 Node 通过真实 ROS Topic 通信。
        output_received_ = false;
        output_value_ = 0;

        publisher_ = nh_.advertise<std_msgs::UInt32>("/comm_lab/test_input", 10);
        subscriber_ = nh_.subscribe(
            "/comm_lab/test_output",
            10,
            &TestableNodeIntegrationTest::outputCallback,
            this);
        // AsyncSpinner 让测试线程可以等待 condition_variable，同时后台仍能接收 ROS callback。
        spinner_.start();

        // 等双方建立 Topic 连接后再发消息，避免“第一条消息在连接建立前丢失”造成偶发失败。
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

    // 用条件变量等待 callback 到来，避免测试用固定 sleep 猜消息什么时候能收到。
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
            // callback 在线程池中执行，因此与测试主线程共享的数据必须用 mutex 保护。
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

// 同时覆盖普通输入和 max_input 边界值。
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

// 非法输入被业务层拒绝时，被测 Node 不应该发布输出消息。
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
