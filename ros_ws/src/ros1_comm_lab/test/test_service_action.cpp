#include <actionlib/client/simple_action_client.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

#include <boost/bind/bind.hpp>
#include <ros1_comm_lab/CountAction.h>
#include <ros1_comm_lab/TransformValue.h>

#include <atomic>
#include <cstdint>

namespace
{

class ServiceActionIntegrationTest : public ::testing::Test
{
protected:
    using ActionClient = actionlib::SimpleActionClient<ros1_comm_lab::CountAction>;

    ServiceActionIntegrationTest()
        : action_client_("/comm_lab/count", true)
    {
    }

    void SetUp() override
    {
        service_client_ = nh_.serviceClient<ros1_comm_lab::TransformValue>(
            "/comm_lab/transform_value");

        ASSERT_TRUE(service_client_.waitForExistence(ros::Duration(2.0)))
            << "transform service did not become available within 2 seconds";
        ASSERT_TRUE(action_client_.waitForServer(ros::Duration(2.0)))
            << "count action server did not become available within 2 seconds";
    }

    void resetFeedback()
    {
        feedback_count_.store(0);
        last_feedback_count_.store(0);
    }

    void feedbackCallback(const ros1_comm_lab::CountFeedbackConstPtr& feedback)
    {
        last_feedback_count_.store(feedback->current_count);
        feedback_count_.fetch_add(1);
    }

    bool waitForActive(double timeout_seconds)
    {
        const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(timeout_seconds);
        while (ros::ok() && ros::WallTime::now() < deadline)
        {
            if (action_client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
            {
                return true;
            }
            ros::WallDuration(0.01).sleep();
        }
        return false;
    }

    ros::NodeHandle nh_;
    ros::ServiceClient service_client_;
    ActionClient action_client_;
    std::atomic<uint32_t> feedback_count_{0};
    std::atomic<uint32_t> last_feedback_count_{0};
};

TEST_F(ServiceActionIntegrationTest, ServiceReturnsTransformedValueForNominalInput)
{
    ros1_comm_lab::TransformValue service;
    service.request.input = 7;

    ASSERT_TRUE(service_client_.call(service));
    EXPECT_TRUE(service.response.success);
    EXPECT_EQ(17u, service.response.output);
    EXPECT_EQ("ok", service.response.message);
}

TEST_F(ServiceActionIntegrationTest, ServiceReportsBusinessRejectionWithoutTransportFailure)
{
    ros1_comm_lab::TransformValue service;
    service.request.input = 101;

    ASSERT_TRUE(service_client_.call(service));
    EXPECT_FALSE(service.response.success);
    EXPECT_EQ(0u, service.response.output);
    EXPECT_EQ("input violates processor constraints", service.response.message);
}

TEST_F(ServiceActionIntegrationTest, ActionCompletesAndPublishesFeedback)
{
    resetFeedback();

    ros1_comm_lab::CountGoal goal;
    goal.target = 5;
    action_client_.sendGoal(
        goal,
        ActionClient::SimpleDoneCallback(),
        ActionClient::SimpleActiveCallback(),
        boost::bind(&ServiceActionIntegrationTest::feedbackCallback, this, boost::placeholders::_1));

    ASSERT_TRUE(action_client_.waitForResult(ros::Duration(2.0)));
    EXPECT_TRUE(action_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);

    const ros1_comm_lab::CountResultConstPtr result = action_client_.getResult();
    ASSERT_TRUE(result);
    EXPECT_TRUE(result->completed);
    EXPECT_EQ(5u, result->final_count);
    EXPECT_EQ("goal completed", result->message);

    EXPECT_GT(feedback_count_.load(), 0u);
    EXPECT_GE(last_feedback_count_.load(), 1u);
    EXPECT_LE(last_feedback_count_.load(), goal.target);
}

TEST_F(ServiceActionIntegrationTest, ActionCancelTransitionsActiveGoalToPreempted)
{
    resetFeedback();

    ros1_comm_lab::CountGoal goal;
    goal.target = 1000;
    action_client_.sendGoal(
        goal,
        ActionClient::SimpleDoneCallback(),
        ActionClient::SimpleActiveCallback(),
        boost::bind(&ServiceActionIntegrationTest::feedbackCallback, this, boost::placeholders::_1));

    ASSERT_TRUE(waitForActive(2.0));
    action_client_.cancelGoal();

    ASSERT_TRUE(action_client_.waitForResult(ros::Duration(2.0)));
    EXPECT_TRUE(action_client_.getState() == actionlib::SimpleClientGoalState::PREEMPTED);

    const ros1_comm_lab::CountResultConstPtr result = action_client_.getResult();
    ASSERT_TRUE(result);
    EXPECT_FALSE(result->completed);
    EXPECT_LT(result->final_count, goal.target);
    EXPECT_EQ("goal preempted", result->message);
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_service_action");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
