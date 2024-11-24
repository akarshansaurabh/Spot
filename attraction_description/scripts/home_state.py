#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.1  # seconds
        self.exit_case = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('JointStatePublisher node has been started')

    def timer_callback(self):
        self.exit_case+=1
        if(self.exit_case<10):
            joint_state_msg = JointState()
            joint_state_msg.header = Header()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = [
                'translate_x', 'translate_y', 'translate_z',
                'rotate_roll', 'rotate_pitch', 'rotate_yaw',
                'front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
                'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
                'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
                'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee',
                'arm_joint1', 'arm_joint2','arm_joint3', 'arm_joint4',
                'arm_joint5', 'arm_joint6','arm_gripper'
            ]
            joint_state_msg.position = [
                0.0, 0.0, 0.175,
                0.0, 0.0, 0.0,
                0.0, 0.785, -1.57,
                0.0, 0.785, -1.57,
                0.0, 0.785, -1.57,
                0.0, 0.785, -1.57,
                1.57, -0.53, 2.35, 0.0,
                0.785, 0.0, 0.0
            ]
            self.publisher_.publish(joint_state_msg)
            self.get_logger().info(f'Published joint states: {joint_state_msg.position}')

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
