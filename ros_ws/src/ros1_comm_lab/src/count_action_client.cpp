#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <ros1_comm_lab/CountAction.h>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{

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

bool parseNonNegativeDouble(const char* text, double& value)
{
    try
    {
        std::size_t parsed = 0;
        value = std::stod(text, &parsed);
        return text[parsed] == '\0' && std::isfinite(value) && value >= 0.0;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

void feedbackCallback(const ros1_comm_lab::CountFeedbackConstPtr& feedback)
{
    std::cout
        << "feedback current_count=" << feedback->current_count
        << " progress=" << feedback->progress << "%"
        << std::endl;
}

}  // namespace

int main(int argc, char** argv)
{
    ros::init(argc, argv, "count_action_client");

    if (argc < 2 || argc > 3)
    {
        std::cerr << "usage: count_action_client <target> [cancel_after_sec]" << std::endl;
        return 2;
    }

    uint32_t target = 0;
    if (!parseUint32(argv[1], target))
    {
        std::cerr << "target must be an unsigned 32-bit integer" << std::endl;
        return 2;
    }

    const bool cancel_requested = argc == 3;
    double cancel_after_sec = 0.0;
    if (cancel_requested && !parseNonNegativeDouble(argv[2], cancel_after_sec))
    {
        std::cerr << "cancel_after_sec must be a non-negative number" << std::endl;
        return 2;
    }

    ros::NodeHandle pnh("~");
    std::string action_name;
    pnh.param<std::string>("action_name", action_name, "/comm_lab/count");

    // The internal spin thread consumes action status/feedback/result callbacks while main waits synchronously.
    actionlib::SimpleActionClient<ros1_comm_lab::CountAction> client(action_name, true);
    if (!client.waitForServer(ros::Duration(2.0)))
    {
        ROS_ERROR_STREAM("action server did not become available within 2 seconds: " << action_name);
        return 3;
    }

    ros1_comm_lab::CountGoal goal;
    goal.target = target;
    client.sendGoal(
        goal,
        actionlib::SimpleActionClient<ros1_comm_lab::CountAction>::SimpleDoneCallback(),
        actionlib::SimpleActionClient<ros1_comm_lab::CountAction>::SimpleActiveCallback(),
        &feedbackCallback);

    if (cancel_requested)
    {
        ros::WallDuration(cancel_after_sec).sleep();
        client.cancelGoal();
    }

    client.waitForResult();

    const actionlib::SimpleClientGoalState state = client.getState();
    const ros1_comm_lab::CountResultConstPtr result = client.getResult();
    if (!result)
    {
        ROS_ERROR_STREAM("action finished without a result, state=" << state.toString());
        return 4;
    }

    std::cout
        << "state=" << state.toString()
        << " completed=" << (result->completed ? 1 : 0)
        << " final_count=" << result->final_count
        << " message=\"" << result->message << "\""
        << std::endl;

    if (!cancel_requested)
    {
        return state == actionlib::SimpleClientGoalState::SUCCEEDED && result->completed ? 0 : 5;
    }

    const bool cancelled =
        state == actionlib::SimpleClientGoalState::PREEMPTED
        || state == actionlib::SimpleClientGoalState::RECALLED;
    return cancelled && !result->completed ? 0 : 5;
}
