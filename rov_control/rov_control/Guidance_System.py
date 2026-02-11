import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
import numpy as np

class DummyROV(Node):
    def __init__(self):
        # ESTA LINHA É A QUE FALTA:
        super().__init__('dummy_rov')
        
        # Parâmetros baseados nos seus dados da UFSC
        self.m = 11.4  
        self.dt = 0.05 # 20Hz de simulação
        
        # Estado atual: [x, y, z, vx, vy, vz]
        self.state = np.zeros(6) 

        # Comunicação
        self.create_subscription(Wrench, '/rov/cmd_wrench', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Agora o create_wall_timer funcionará pois a classe Node foi inicializada
        self.timer = self.create_timer(self.dt, self.update_physics)

    def cmd_callback(self, msg):
        force = np.array([msg.force.x, msg.force.y, msg.force.z])
        accel = force / self.m
        self.state[3:6] += accel * self.dt

    def update_physics(self):
        self.state[0:3] += self.state[3:6] * self.dt

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.pose.pose.position.x = float(self.state[0])
        odom.pose.pose.position.y = float(self.state[1])
        odom.pose.pose.position.z = float(self.state[2])
        odom.pose.pose.orientation.w = 1.0
        
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = DummyROV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
