#!/usr/bin/env python3
"""
multiModeQuad_ROS 多机控制脚本

用法:
  python3 multi_quad_demo.py                    # 默认控制 quad_1
  python3 multi_quad_demo.py --namespace quad_2 # 控制 quad_2
  python3 multi_quad_demo.py --dual             # 同时控制 quad_1 和 quad_2

依赖:
  pip install rospy (ROS Noetic 自带)
"""

import argparse
import math
import rospy
from std_msgs.msg import Int16, Float32
from geometry_msgs.msg import Twist, PoseStamped, TwistStamped


class QuadController:
    """四旋翼 ROS 控制器"""

    def __init__(self, namespace="quad_1"):
        self.ns = namespace
        rospy.loginfo(f"[{namespace}] Initializing controller...")

        # 发布器
        self.pub_flight_mode = rospy.Publisher(
            f"/{namespace}/flight_mode", Int16, queue_size=1)
        self.pub_velocity = rospy.Publisher(
            f"/{namespace}/setpoint_velocity/cmd_vel_unstamped",
            Twist, queue_size=1)
        self.pub_attitude = rospy.Publisher(
            f"/{namespace}/setpoint_attitude/attitude",
            PoseStamped, queue_size=1)
        self.pub_rate = rospy.Publisher(
            f"/{namespace}/setpoint_attitude/cmd_vel",
            TwistStamped, queue_size=1)
        self.pub_thrust = rospy.Publisher(
            f"/{namespace}/thrust", Float32, queue_size=1)

        # 状态
        self.mode = 0
        self.time = 0.0

    def set_mode(self, mode: int):
        """设置飞行模式"""
        self.mode = mode
        self.pub_flight_mode.publish(Int16(data=mode))
        rospy.loginfo(f"[{self.ns}] Mode set to {mode}")

    def cmd_velocity(self, vx=0.0, vy=0.0, vz=0.0, wx=0.0, wy=0.0, wz=0.0):
        """发送速度指令（机体坐标系）"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = wx
        twist.angular.y = wy
        twist.angular.z = wz
        self.pub_velocity.publish(twist)

    def cmd_attitude(self, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """发送姿态指令（四元数）"""
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub_attitude.publish(msg)

    def cmd_rate(self, wx=0.0, wy=0.0, wz=0.0):
        """发送角速率指令"""
        msg = TwistStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.twist.angular.x = wx
        msg.twist.angular.y = wy
        msg.twist.angular.z = wz
        self.pub_rate.publish(msg)

    def cmd_thrust(self, thrust: float):
        """发送推力指令"""
        self.pub_thrust.publish(Float32(data=thrust))


def circular_trajectory(controller: QuadController, radius=2.0, height=1.0,
                        omega=0.3, rate=20):
    """圆形轨迹跟踪演示（速度控制模式）"""
    t = 0.0
    while not rospy.is_shutdown():
        # 圆形轨迹速度指令（体轴系）
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        controller.cmd_velocity(vx=vx, vy=vy, vz=-0.2)  # NED: 上升为负
        t += 1.0 / rate
        rospy.sleep(1.0 / rate)


def hover_demo(controller: QuadController, duration=10):
    """悬停演示"""
    controller.set_mode(0)
    rospy.loginfo(f"[{controller.ns}] Hovering for {duration}s...")
    for i in range(duration * 20):
        if rospy.is_shutdown():
            break
        controller.cmd_velocity(vx=0, vy=0, vz=0)
        rospy.sleep(0.05)


def dual_control():
    """双机独立控制演示"""
    q1 = QuadController("quad_1")
    q2 = QuadController("quad_2")

    # 等待连接
    rospy.sleep(1.0)

    q1.set_mode(0)
    q2.set_mode(0)
    rospy.sleep(0.5)

    rospy.loginfo("=== quad_1: forward flight ===")
    rospy.loginfo("=== quad_2: circular trajectory ===")

    rate = rospy.Rate(20)
    t = 0.0
    while not rospy.is_shutdown():
        # quad_1: 前飞 + 上升
        q1.cmd_velocity(vx=1.0, vz=-0.3)

        # quad_2: 圆形轨迹
        radius = 1.5
        omega = 0.3
        vx = -radius * omega * math.sin(omega * t)
        vy = radius * omega * math.cos(omega * t)
        q2.cmd_velocity(vx=vx, vy=vy, vz=0)

        t += 0.05
        rate.sleep()


def main():
    parser = argparse.ArgumentParser(description="multiModeQuad_ROS 控制脚本")
    parser.add_argument("--namespace", default="quad_1", help="四旋翼命名空间")
    parser.add_argument("--dual", action="store_true", help="双机控制模式")
    parser.add_argument("--mode", type=int, default=0, choices=[0, 1, 2],
                        help="飞行模式 (0=速度, 1=角速率, 2=姿态)")
    parser.add_argument("--trajectory", choices=["hover", "circle", "forward"],
                        default="hover", help="轨迹类型")
    args = parser.parse_args()

    rospy.init_node("multi_quad_demo", anonymous=True)

    if args.dual:
        dual_control()
        return

    controller = QuadController(args.namespace)
    rospy.sleep(1.0)  # 等待连接

    controller.set_mode(args.mode)
    rospy.sleep(0.5)

    if args.trajectory == "hover":
        hover_demo(controller)
    elif args.trajectory == "circle":
        circular_trajectory(controller)
    elif args.trajectory == "forward":
        rospy.loginfo(f"[{args.namespace}] Forward flight...")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            controller.cmd_velocity(vx=1.0, vz=-0.3)
            rate.sleep()


if __name__ == "__main__":
    main()
