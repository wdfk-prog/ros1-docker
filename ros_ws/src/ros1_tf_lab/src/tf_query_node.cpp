#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <stdexcept>
#include <string>

namespace ros1_tf_lab
{

class TfQueryNode
{
public:
    TfQueryNode()
        : private_nh_("~"), tf_listener_(tf_buffer_)
    {
        private_nh_.param<std::string>("target_frame", target_frame_, "map");
        private_nh_.param<std::string>("source_frame", source_frame_, "laser_link");
        private_nh_.param<std::string>("query_mode", query_mode_, "latest");
        private_nh_.param("query_offset_sec", query_offset_sec_, 0.0);
        private_nh_.param("query_rate_hz", query_rate_hz_, 2.0);
        private_nh_.param("timeout_sec", timeout_sec_, 0.05);

        validateParameters();

        timer_ = nh_.createTimer(ros::Duration(1.0 / query_rate_hz_), &TfQueryNode::timerCallback, this);

        ROS_INFO_STREAM("tf_query_node target=" << target_frame_ << " source=" << source_frame_
                                                 << " mode=" << query_mode_
                                                 << " offset=" << query_offset_sec_ << " s");
    }

private:
    void validateParameters() const
    {
        if (target_frame_.empty() || source_frame_.empty())
        {
            throw std::runtime_error("target_frame and source_frame must not be empty");
        }

        if (query_mode_ != "latest" && query_mode_ != "now")
        {
            throw std::runtime_error("query_mode must be 'latest' or 'now'");
        }

        if (!std::isfinite(query_offset_sec_))
        {
            throw std::runtime_error("query_offset_sec must be finite");
        }

        if (!std::isfinite(query_rate_hz_) || query_rate_hz_ <= 0.0)
        {
            throw std::runtime_error("query_rate_hz must be a finite positive value");
        }

        if (!std::isfinite(timeout_sec_) || timeout_sec_ < 0.0)
        {
            throw std::runtime_error("timeout_sec must be a finite non-negative value");
        }
    }

    ros::Time queryTime() const
    {
        if (query_mode_ == "latest")
        {
            // Time zero asks tf2 for the latest common time available across the complete transform chain.
            return ros::Time(0);
        }

        // The offset is intentionally configurable so the lab can request past/future times and expose
        // extrapolation behavior without changing the production timestamp carried by the broadcaster.
        return ros::Time::now() + ros::Duration(query_offset_sec_);
    }

    void timerCallback(const ros::TimerEvent &)
    {
        const ros::Time requested_time = queryTime();

        try
        {
            const geometry_msgs::TransformStamped transform = tf_buffer_.lookupTransform(
                target_frame_, source_frame_, requested_time, ros::Duration(timeout_sec_));

            ROS_INFO_STREAM_THROTTLE(
                1.0,
                "TF OK " << transform.header.frame_id << " <- " << transform.child_frame_id
                          << " stamp=" << transform.header.stamp
                          << " xyz=[" << transform.transform.translation.x << ", "
                          << transform.transform.translation.y << ", "
                          << transform.transform.translation.z << "]");
        }
        catch (const tf2::LookupException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "TF LookupException: " << ex.what());
        }
        catch (const tf2::ConnectivityException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "TF ConnectivityException: " << ex.what());
        }
        catch (const tf2::ExtrapolationException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "TF ExtrapolationException: " << ex.what());
        }
        catch (const tf2::TransformException &ex)
        {
            ROS_WARN_STREAM_THROTTLE(1.0, "TF TransformException: " << ex.what());
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Timer timer_;

    std::string target_frame_;
    std::string source_frame_;
    std::string query_mode_;
    double query_offset_sec_ = 0.0;
    double query_rate_hz_ = 2.0;
    double timeout_sec_ = 0.05;
};

}  // namespace ros1_tf_lab

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_query_node");

    try
    {
        ros1_tf_lab::TfQueryNode node;
        ros::spin();
    }
    catch (const std::exception &ex)
    {
        ROS_FATAL_STREAM("tf_query_node startup failed: " << ex.what());
        return 1;
    }

    return 0;
}
