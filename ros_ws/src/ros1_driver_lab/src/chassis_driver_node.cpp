#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

namespace
{
constexpr double kGravityMps2 = 9.80665;

/**
 * @brief 底层设备已经取得的一帧底盘数据。
 *
 * 第 12 章故意不讨论这些数据通过 TCP、CAN、串口还是厂商 SDK 获得。
 * ROS Driver 从这一层开始，只关心“物理量是什么、单位是什么”。
 */
struct ChassisData
{
    double left_wheel_position_rad = 0.0;
    double right_wheel_position_rad = 0.0;
    double left_wheel_velocity_rad_s = 0.0;
    double right_wheel_velocity_rad_s = 0.0;

    double imu_angular_velocity_z_rad_s = 0.0;
    double imu_linear_acceleration_x_m_s2 = 0.0;
    double imu_linear_acceleration_y_m_s2 = 0.0;
    double imu_linear_acceleration_z_m_s2 = kGravityMps2;

    bool online = true;
};

/**
 * @brief ROS 的 /cmd_vel 经过差速运动学后得到的左右轮目标角速度。
 */
struct WheelCommand
{
    double left_wheel_velocity_rad_s = 0.0;
    double right_wheel_velocity_rad_s = 0.0;
};

/**
 * @brief 极简教学设备。
 *
 * 真实项目中，这个类的位置可以由真实硬件驱动、SDK 或 transport 层替换。
 * 本章只需要一个可运行的数据源，因此它直接在进程内累计轮角度并返回 ChassisData。
 */
class DemoChassisDevice
{
public:
    DemoChassisDevice(double wheel_radius, double wheel_separation)
        : wheel_radius_(wheel_radius), wheel_separation_(wheel_separation)
    {
    }

    /**
     * @brief 保存 Driver 计算出的左右轮目标角速度。
     */
    void writeWheelCommand(const WheelCommand &command)
    {
        wheel_command_ = command;
    }

    /**
     * @brief 生成一帧已经取得的底盘数据。
     * @param dt_s 距离上一帧的时间，单位 s。
     * @return 当前底盘数据。
     */
    ChassisData readChassisData(double dt_s)
    {
        // 教学设备假设“实际轮速 = 目标轮速”。真实硬件中这里应该来自编码器反馈。
        data_.left_wheel_velocity_rad_s = wheel_command_.left_wheel_velocity_rad_s;
        data_.right_wheel_velocity_rad_s = wheel_command_.right_wheel_velocity_rad_s;

        // 角位置 = 对角速度按时间积分。JointState.position 后面直接使用这个累计角度。
        data_.left_wheel_position_rad += data_.left_wheel_velocity_rad_s * dt_s;
        data_.right_wheel_position_rad += data_.right_wheel_velocity_rad_s * dt_s;

        // 用左右轮速度反算底盘角速度，作为教学 IMU 的 gyro_z 数据。
        const double left_linear_m_s = data_.left_wheel_velocity_rad_s * wheel_radius_;
        const double right_linear_m_s = data_.right_wheel_velocity_rad_s * wheel_radius_;
        data_.imu_angular_velocity_z_rad_s =
            (right_linear_m_s - left_linear_m_s) / wheel_separation_;

        // 为了让 Imu 消息字段可观察，示例假设 imu_link 为 x 前/y 左/z 上。
        // 同时假设机器人平放且没有额外线加速度。
        data_.imu_linear_acceleration_x_m_s2 = 0.0;
        data_.imu_linear_acceleration_y_m_s2 = 0.0;
        data_.imu_linear_acceleration_z_m_s2 = kGravityMps2;
        data_.online = true;

        return data_;
    }

private:
    double wheel_radius_;
    double wheel_separation_;
    WheelCommand wheel_command_;
    ChassisData data_;
};

class ChassisDriverNode
{
public:
    ChassisDriverNode()
        : nh_(), private_nh_("~")
    {
        loadParameters();
        validateParameters();

        device_.reset(new DemoChassisDevice(wheel_radius_, wheel_separation_));

        cmd_vel_sub_ = nh_.subscribe(cmd_vel_topic_, 10, &ChassisDriverNode::cmdVelCallback, this);
        joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_states_topic_, 10);
        imu_pub_ = nh_.advertise<sensor_msgs::Imu>(imu_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 10);
        diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(diagnostics_topic_, 10);
    }

    void run()
    {
        ros::Rate rate(publish_rate_hz_);
        ros::Time previous_stamp = ros::Time::now();

        while (ros::ok())
        {
            ros::spinOnce();

            const ros::Time stamp = ros::Time::now();
            double dt_s = (stamp - previous_stamp).toSec();
            previous_stamp = stamp;

            // 第一次循环或系统调度异常时避免把无效 dt 注入积分。
            if (!std::isfinite(dt_s) || dt_s <= 0.0)
            {
                dt_s = 1.0 / publish_rate_hz_;
            }

            // 从这一行开始，把底层设备数据当成“已经获取好”。
            const ChassisData data = device_->readChassisData(dt_s);

            publishJointState(data, stamp);
            publishImu(data, stamp);
            publishOdometry(data, stamp, dt_s);
            publishDiagnostics(data, stamp);

            rate.sleep();
        }
    }

private:
    void loadParameters()
    {
        private_nh_.param("wheel_radius", wheel_radius_, 0.10);
        private_nh_.param("wheel_separation", wheel_separation_, 0.50);
        private_nh_.param("publish_rate_hz", publish_rate_hz_, 20.0);

        private_nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, "/cmd_vel");
        private_nh_.param<std::string>("joint_states_topic", joint_states_topic_, "/joint_states");
        private_nh_.param<std::string>("imu_topic", imu_topic_, "/imu/data_raw");
        private_nh_.param<std::string>("odom_topic", odom_topic_, "/odom");
        private_nh_.param<std::string>("diagnostics_topic", diagnostics_topic_, "/diagnostics");

        private_nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
        private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
        private_nh_.param<std::string>("imu_frame_id", imu_frame_id_, "imu_link");
        private_nh_.param<std::string>("left_joint_name", left_joint_name_, "left_wheel_joint");
        private_nh_.param<std::string>("right_joint_name", right_joint_name_, "right_wheel_joint");

        private_nh_.param("imu_angular_velocity_variance", imu_angular_velocity_variance_, 0.001);
        private_nh_.param("imu_linear_acceleration_variance", imu_linear_acceleration_variance_, 0.04);
        private_nh_.param("odom_pose_xy_variance", odom_pose_xy_variance_, 0.02);
        private_nh_.param("odom_pose_yaw_variance", odom_pose_yaw_variance_, 0.05);
        private_nh_.param("odom_twist_linear_variance", odom_twist_linear_variance_, 0.01);
        private_nh_.param("odom_twist_angular_variance", odom_twist_angular_variance_, 0.02);
        private_nh_.param("unobserved_variance", unobserved_variance_, 1000000.0);
    }

    void validateParameters() const
    {
        if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0)
        {
            throw std::runtime_error("wheel_radius must be a finite positive value");
        }
        if (!std::isfinite(wheel_separation_) || wheel_separation_ <= 0.0)
        {
            throw std::runtime_error("wheel_separation must be a finite positive value");
        }
        if (!std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0)
        {
            throw std::runtime_error("publish_rate_hz must be a finite positive value");
        }

        // covariance 表示方差/协方差，负数或 NaN/Inf 都没有合法物理意义。
        validateVariance("imu_angular_velocity_variance", imu_angular_velocity_variance_);
        validateVariance("imu_linear_acceleration_variance", imu_linear_acceleration_variance_);
        validateVariance("odom_pose_xy_variance", odom_pose_xy_variance_);
        validateVariance("odom_pose_yaw_variance", odom_pose_yaw_variance_);
        validateVariance("odom_twist_linear_variance", odom_twist_linear_variance_);
        validateVariance("odom_twist_angular_variance", odom_twist_angular_variance_);
        validateVariance("unobserved_variance", unobserved_variance_);
    }

    static void validateVariance(const char *name, double value)
    {
        if (!std::isfinite(value) || value < 0.0)
        {
            throw std::runtime_error(std::string(name) + " must be a finite non-negative value");
        }
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        // 差速底盘只消费前进线速度 linear.x 和绕 Z 轴角速度 angular.z。
        if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z))
        {
            ROS_WARN_THROTTLE(1.0, "ignore non-finite /cmd_vel command");
            return;
        }

        if (msg->linear.y != 0.0 || msg->linear.z != 0.0 || msg->angular.x != 0.0 || msg->angular.y != 0.0)
        {
            ROS_WARN_THROTTLE(2.0, "differential chassis only uses Twist.linear.x and Twist.angular.z");
        }

        // 逆运动学：先得到左右轮轮缘线速度，再除以车轮半径得到轮轴角速度。
        const double left_linear_m_s = msg->linear.x - msg->angular.z * wheel_separation_ / 2.0;
        const double right_linear_m_s = msg->linear.x + msg->angular.z * wheel_separation_ / 2.0;

        WheelCommand command;
        command.left_wheel_velocity_rad_s = left_linear_m_s / wheel_radius_;
        command.right_wheel_velocity_rad_s = right_linear_m_s / wheel_radius_;

        // 本章故意不实现命令超时/watchdog。真实底盘不能无限保持最后一条速度命令，
        // 这部分属于 Driver 可靠性专项，不在当前 Navigation 学习主线展开。
        device_->writeWheelCommand(command);

        ROS_INFO_THROTTLE(1.0,
                          "cmd_vel: v=%.3f m/s, w=%.3f rad/s -> left=%.3f rad/s, right=%.3f rad/s",
                          msg->linear.x,
                          msg->angular.z,
                          command.left_wheel_velocity_rad_s,
                          command.right_wheel_velocity_rad_s);
    }

    void publishJointState(const ChassisData &data, const ros::Time &stamp)
    {
        sensor_msgs::JointState msg;
        msg.header.stamp = stamp;

        // name / position / velocity 使用相同下标一一对应左右轮。
        msg.name = {left_joint_name_, right_joint_name_};
        msg.position = {data.left_wheel_position_rad, data.right_wheel_position_rad};
        msg.velocity = {data.left_wheel_velocity_rad_s, data.right_wheel_velocity_rad_s};

        // 当前示例没有力矩传感器，因此 effort 留空，而不是伪造 0 Nm 测量值。
        joint_state_pub_.publish(msg);
    }

    void publishImu(const ChassisData &data, const ros::Time &stamp)
    {
        sensor_msgs::Imu msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = imu_frame_id_;

        // 当前示例没有姿态解算结果。orientation_covariance[0] = -1 表示不要使用 orientation。
        msg.orientation.w = 1.0;
        msg.orientation_covariance[0] = -1.0;

        msg.angular_velocity.z = data.imu_angular_velocity_z_rad_s;
        msg.angular_velocity_covariance[0] = unobserved_variance_;
        msg.angular_velocity_covariance[4] = unobserved_variance_;
        msg.angular_velocity_covariance[8] = imu_angular_velocity_variance_;

        msg.linear_acceleration.x = data.imu_linear_acceleration_x_m_s2;
        msg.linear_acceleration.y = data.imu_linear_acceleration_y_m_s2;
        msg.linear_acceleration.z = data.imu_linear_acceleration_z_m_s2;
        msg.linear_acceleration_covariance[0] = imu_linear_acceleration_variance_;
        msg.linear_acceleration_covariance[4] = imu_linear_acceleration_variance_;
        msg.linear_acceleration_covariance[8] = imu_linear_acceleration_variance_;

        imu_pub_.publish(msg);
    }

    void publishOdometry(const ChassisData &data, const ros::Time &stamp, double dt_s)
    {
        // 正运动学：轮轴角速度 * 轮半径 = 轮缘线速度。
        const double left_linear_m_s = data.left_wheel_velocity_rad_s * wheel_radius_;
        const double right_linear_m_s = data.right_wheel_velocity_rad_s * wheel_radius_;

        // 差速底盘中心线速度等于左右轮线速度平均值。
        const double linear_m_s = (right_linear_m_s + left_linear_m_s) / 2.0;

        // 左右轮速度差决定绕 Z 轴角速度。
        const double angular_rad_s = (right_linear_m_s - left_linear_m_s) / wheel_separation_;

        // 用当前 yaw 把车体前向速度投影到 odom 坐标系，再进行简单欧拉积分。
        odom_x_m_ += linear_m_s * std::cos(odom_yaw_rad_) * dt_s;
        odom_y_m_ += linear_m_s * std::sin(odom_yaw_rad_) * dt_s;
        odom_yaw_rad_ += angular_rad_s * dt_s;

        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = odom_frame_id_;
        msg.child_frame_id = base_frame_id_;

        msg.pose.pose.position.x = odom_x_m_;
        msg.pose.pose.position.y = odom_y_m_;

        // 二维底盘只有 yaw，这里把 yaw 转成绕 Z 轴旋转的四元数。
        msg.pose.pose.orientation.z = std::sin(odom_yaw_rad_ / 2.0);
        msg.pose.pose.orientation.w = std::cos(odom_yaw_rad_ / 2.0);

        msg.twist.twist.linear.x = linear_m_s;
        msg.twist.twist.angular.z = angular_rad_s;

        fillOdometryCovariance(msg);
        odom_pub_.publish(msg);
    }

    void fillOdometryCovariance(nav_msgs::Odometry &msg) const
    {
        // 6x6 covariance 的顺序是 x, y, z, roll, pitch, yaw。
        // 对角线索引因此是 0, 7, 14, 21, 28, 35。
        msg.pose.covariance[0] = odom_pose_xy_variance_;
        msg.pose.covariance[7] = odom_pose_xy_variance_;
        msg.pose.covariance[14] = unobserved_variance_;
        msg.pose.covariance[21] = unobserved_variance_;
        msg.pose.covariance[28] = unobserved_variance_;
        msg.pose.covariance[35] = odom_pose_yaw_variance_;

        msg.twist.covariance[0] = odom_twist_linear_variance_;
        msg.twist.covariance[7] = unobserved_variance_;
        msg.twist.covariance[14] = unobserved_variance_;
        msg.twist.covariance[21] = unobserved_variance_;
        msg.twist.covariance[28] = unobserved_variance_;
        msg.twist.covariance[35] = odom_twist_angular_variance_;
    }

    void publishDiagnostics(const ChassisData &data, const ros::Time &stamp)
    {
        diagnostic_msgs::DiagnosticArray array;
        array.header.stamp = stamp;

        diagnostic_msgs::DiagnosticStatus status;
        status.name = "ros1_driver_lab/demo_chassis";
        status.hardware_id = "demo_chassis";
        status.level = data.online ? diagnostic_msgs::DiagnosticStatus::OK : diagnostic_msgs::DiagnosticStatus::ERROR;
        status.message = data.online ? "device data available" : "device offline";

        diagnostic_msgs::KeyValue source;
        source.key = "data_source";
        source.value = "in-process demo; transport intentionally omitted in chapter 12";
        status.values.push_back(source);

        array.status.push_back(status);
        diagnostics_pub_.publish(array);
    }

    // ROS 通信对象：一个速度命令订阅者和四个标准消息发布者。
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber cmd_vel_sub_;
    ros::Publisher joint_state_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher diagnostics_pub_;

    // 教学设备只负责提供/接收结构化物理量，不承担任何网络协议教学。
    std::unique_ptr<DemoChassisDevice> device_;

    // 底盘几何参数与发布频率。
    double wheel_radius_ = 0.10;
    double wheel_separation_ = 0.50;
    double publish_rate_hz_ = 20.0;

    // Topic、frame 和 joint 名称全部从 YAML 参数加载。
    std::string cmd_vel_topic_;
    std::string joint_states_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string diagnostics_topic_;
    std::string odom_frame_id_;
    std::string base_frame_id_;
    std::string imu_frame_id_;
    std::string left_joint_name_;
    std::string right_joint_name_;

    // covariance 教学参数。真实产品应由规格、统计或标定确定。
    double imu_angular_velocity_variance_ = 0.001;
    double imu_linear_acceleration_variance_ = 0.04;
    double odom_pose_xy_variance_ = 0.02;
    double odom_pose_yaw_variance_ = 0.05;
    double odom_twist_linear_variance_ = 0.01;
    double odom_twist_angular_variance_ = 0.02;
    double unobserved_variance_ = 1000000.0;

    // 最基础的二维里程计状态。第 13 章再处理时间戳、延迟和 jitter。
    double odom_x_m_ = 0.0;
    double odom_y_m_ = 0.0;
    double odom_yaw_rad_ = 0.0;
};
}  // namespace

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chassis_driver_node");

    try
    {
        ChassisDriverNode node;
        node.run();
    }
    catch (const std::exception &e)
    {
        ROS_FATAL("chassis_driver_node startup failed: %s", e.what());
        return 1;
    }

    return 0;
}
