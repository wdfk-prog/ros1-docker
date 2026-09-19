#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def cross2(ax, ay, bx, by):
    return ax * by - ay * bx


def ray_segment_distance(origin_x, origin_y, dir_x, dir_y, segment):
    x1, y1, x2, y2 = segment
    seg_x = x2 - x1
    seg_y = y2 - y1
    denominator = cross2(dir_x, dir_y, seg_x, seg_y)
    if abs(denominator) < 1e-12:
        return None

    rel_x = x1 - origin_x
    rel_y = y1 - origin_y
    ray_t = cross2(rel_x, rel_y, seg_x, seg_y) / denominator
    seg_u = cross2(rel_x, rel_y, dir_x, dir_y) / denominator
    if ray_t < 0.0 or seg_u < 0.0 or seg_u > 1.0:
        return None
    return ray_t


class SimpleLaserWorld:
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.laser_frame_id = rospy.get_param("~laser_frame_id", "laser_link")

        self.scan_rate_hz = float(rospy.get_param("~scan_rate_hz", 10.0))
        self.integration_rate_hz = float(rospy.get_param("~integration_rate_hz", 100.0))
        self.beam_count = int(rospy.get_param("~beam_count", 360))
        self.angle_min = float(rospy.get_param("~angle_min_rad", -math.pi))
        self.range_min = float(rospy.get_param("~range_min_m", 0.05))
        self.range_max = float(rospy.get_param("~range_max_m", 8.0))

        self.laser_offset_x = float(rospy.get_param("~laser_offset_x_m", 0.25))
        self.laser_offset_y = float(rospy.get_param("~laser_offset_y_m", 0.0))
        self.laser_yaw_offset = float(rospy.get_param("~laser_yaw_offset_rad", 0.0))
        self.truth_linear_scale = float(rospy.get_param("~truth_linear_scale", 1.03))
        self.truth_angular_scale = float(rospy.get_param("~truth_angular_scale", 1.0))

        self.x = float(rospy.get_param("~initial_x_m", 0.0))
        self.y = float(rospy.get_param("~initial_y_m", 0.0))
        self.yaw = float(rospy.get_param("~initial_yaw_rad", 0.0))
        self.segments = [tuple(map(float, item)) for item in rospy.get_param("~segments")]

        if self.scan_rate_hz <= 0.0 or self.integration_rate_hz <= 0.0:
            raise ValueError("scan_rate_hz and integration_rate_hz must be positive")
        if self.beam_count < 2:
            raise ValueError("beam_count must be at least 2")
        if self.range_min <= 0.0 or self.range_max <= self.range_min:
            raise ValueError("require 0 < range_min_m < range_max_m")
        if any(len(segment) != 4 for segment in self.segments):
            raise ValueError("each segment must be [x1, y1, x2, y2]")

        self.angle_increment = 2.0 * math.pi / self.beam_count
        self.angle_max = self.angle_min + (self.beam_count - 1) * self.angle_increment
        self.linear_cmd = 0.0
        self.angular_cmd = 0.0
        self.last_integrate_stamp = rospy.Time.now()

        self.scan_pub = rospy.Publisher(self.scan_topic, LaserScan, queue_size=5)
        self.cmd_sub = rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_callback, queue_size=10)
        self.integration_timer = rospy.Timer(
            rospy.Duration(1.0 / self.integration_rate_hz), self.integrate_callback
        )
        self.scan_timer = rospy.Timer(rospy.Duration(1.0 / self.scan_rate_hz), self.scan_callback)

        rospy.loginfo(
            "simple laser world: %d beams, %.1f Hz, truth scales linear=%.3f angular=%.3f",
            self.beam_count,
            self.scan_rate_hz,
            self.truth_linear_scale,
            self.truth_angular_scale,
        )

    def cmd_vel_callback(self, msg):
        if not math.isfinite(msg.linear.x) or not math.isfinite(msg.angular.z):
            rospy.logwarn_throttle(1.0, "ignore non-finite /cmd_vel")
            return
        self.linear_cmd = msg.linear.x
        self.angular_cmd = msg.angular.z

    def integrate_callback(self, event):
        stamp = event.current_real
        dt = (stamp - self.last_integrate_stamp).to_sec()
        self.last_integrate_stamp = stamp
        if not math.isfinite(dt) or dt <= 0.0:
            return

        # 与第 12 章教学 Driver 保持相同命令语义：持续使用最后一条有效 Twist，直到收到下一条命令。
        linear = self.linear_cmd
        angular = self.angular_cmd
        linear *= self.truth_linear_scale
        angular *= self.truth_angular_scale

        self.x += linear * math.cos(self.yaw) * dt
        self.y += linear * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + angular * dt), math.cos(self.yaw + angular * dt))

    def scan_callback(self, _event):
        stamp = rospy.Time.now()
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        laser_x = self.x + cos_yaw * self.laser_offset_x - sin_yaw * self.laser_offset_y
        laser_y = self.y + sin_yaw * self.laser_offset_x + cos_yaw * self.laser_offset_y
        laser_yaw = self.yaw + self.laser_yaw_offset

        ranges = []
        for index in range(self.beam_count):
            local_angle = self.angle_min + index * self.angle_increment
            world_angle = laser_yaw + local_angle
            dir_x = math.cos(world_angle)
            dir_y = math.sin(world_angle)

            nearest = self.range_max + 1.0
            for segment in self.segments:
                distance = ray_segment_distance(laser_x, laser_y, dir_x, dir_y, segment)
                if distance is not None and distance < nearest:
                    nearest = distance

            if nearest < self.range_min or nearest > self.range_max:
                ranges.append(float("inf"))
            else:
                ranges.append(nearest)

        msg = LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self.laser_frame_id
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.angle_increment
        msg.time_increment = 0.0  # 教学仿真把整帧视为同一时刻；真实旋转雷达通常不是 0。
        msg.scan_time = 1.0 / self.scan_rate_hz
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = []
        self.scan_pub.publish(msg)


def main():
    rospy.init_node("simple_laser_world")
    SimpleLaserWorld()
    rospy.spin()


if __name__ == "__main__":
    main()
