#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <string>

namespace ros1_tf_lab
{

class OdomTfBroadcaster
{
public:
    OdomTfBroadcaster()
        : private_nh_("~")
    {
        private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        odom_sub_ = nh_.subscribe(odom_topic_, 20, &OdomTfBroadcaster::odomCallback, this);

        ROS_INFO_STREAM("odom_tf_broadcaster subscribes to " << odom_topic_);
    }

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    {
        if (msg->header.frame_id.empty())
        {
            ROS_WARN_THROTTLE(2.0, "Ignore Odometry: header.frame_id is empty");
            return;
        }

        if (msg->child_frame_id.empty())
        {
            ROS_WARN_THROTTLE(2.0, "Ignore Odometry: child_frame_id is empty");
            return;
        }

        if (msg->header.frame_id == msg->child_frame_id)
        {
            ROS_WARN_THROTTLE(2.0, "Ignore Odometry: parent and child frame are identical");
            return;
        }

        geometry_msgs::TransformStamped transform;

        // Preserve the Odometry sample time. Replacing it with ros::Time::now() would relabel delayed data
        // as current and break time-aligned TF queries made by downstream sensor consumers.
        transform.header.stamp = msg->header.stamp;
        transform.header.frame_id = msg->header.frame_id;
        transform.child_frame_id = msg->child_frame_id;
        transform.transform.translation.x = msg->pose.pose.position.x;
        transform.transform.translation.y = msg->pose.pose.position.y;
        transform.transform.translation.z = msg->pose.pose.position.z;
        transform.transform.rotation = msg->pose.pose.orientation;

        broadcaster_.sendTransform(transform);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    tf2_ros::TransformBroadcaster broadcaster_;
    std::string odom_topic_;
};

}  // namespace ros1_tf_lab

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_tf_broadcaster");
    ros1_tf_lab::OdomTfBroadcaster node;
    ros::spin();
    return 0;
}
