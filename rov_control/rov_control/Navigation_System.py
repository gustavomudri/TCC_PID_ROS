import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import random

class RandomTrajectoryNode(Node):
    def __init__(self):
        # ESTA LINHA É ESSENCIAL:
        super().__init__('trajectory_node')
        
        self.publisher = self.create_publisher(Odometry, '/rov/desired_state', 10)
        
        # O rclpy usa create_timer ou create_wall_timer. 
        # No Python 3.12/Humble, garanta que o primeiro argumento seja float.
        self.timer = self.create_timer(0.1, self.generate_step) 
        
        self.curr_x, self.curr_y, self.curr_z = 0.0, 0.0, -1.0

    def generate_step(self):
        self.get_logger().info(f'Publicando alvo: x={self.curr_x:.2f}')
        self.curr_x += random.uniform(-0.04, 0.04)
        self.curr_y += random.uniform(-0.04, 0.04)
        self.curr_z += random.uniform(-0.02, 0.02)

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.pose.position.x = float(self.curr_x)
        msg.pose.pose.position.y = float(self.curr_y)
        msg.pose.pose.position.z = float(self.curr_z)
        
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RandomTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
