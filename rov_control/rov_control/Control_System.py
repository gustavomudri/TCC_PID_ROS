import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Wrench
from scipy.spatial.transform import Rotation as R

class EulerLagrangeController(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Parâmetros Físicos Revisados
        self.m = 11.4; self.zg = 0.02; self.g = 9.82 
        self.W = self.m * self.g 
        self.B = 1000.0 * 0.0114 * self.g # Baseado no volume de 0.0114 m3
        
        # Ganhos PID (Ajustáveis conforme a resposta do simulador)
        self.Kp = np.array([45.0, 45.0, 70.0, 15.0, 15.0, 15.0])
        self.Kd = np.array([25.0, 25.0, 35.0, 8.0, 8.0, 8.0])

        self.pose_d = np.zeros(6) 
        self.pose_curr = np.zeros(6) 
        self.vel_curr = np.zeros(6)

        # Subscrições e Publicações
        self.create_subscription(Odometry, '/rov/desired_state', self.traj_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.publisher = self.create_publisher(Wrench, '/rov/cmd_wrench', 10)

    def traj_callback(self, msg):
        # Recebe a posição desejada do nó de trajetória
        self.pose_d[0:3] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]

    def odom_callback(self, msg):
        self.get_logger().info('Odometria recebida! Calculando Euler-Lagrange...')
        # 1. Estado Atual
        self.pose_curr[0:3] = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        self.vel_curr[0:6] = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z,
                             msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z]
        
        # 2. Orientação para cálculo de G(eta)
        q = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        rot_matrix = R.from_quat(q).as_matrix()

        # 3. Termos de Euler-Lagrange
        g_eta = self.calculate_g(rot_matrix)
        # Arrasto (Drag) usando os coeficientes da sua tabela
        d_nu = 17.77 * np.abs(self.vel_curr) * self.vel_curr + 25.15 * self.vel_curr

        # 4. Lei de Controle: Compensação + Ação de Correção
        error = self.pose_d - self.pose_curr
        tau = g_eta + d_nu + (self.Kp * error) - (self.Kd * self.vel_curr)

        # 5. Publicação
        cmd = Wrench()
        cmd.force.x, cmd.force.y, cmd.force.z = tau[0:3]
        cmd.torque.x, cmd.torque.y, cmd.torque.z = tau[3:6]
        self.publisher.publish(cmd)

    def calculate_g(self, R_mat):
        # Simplificação do vetor de forças restauradoras para ROV estável
        sin_theta = -R_mat[2, 0]
        return np.array([(self.W-self.B)*sin_theta, 0, -(self.W-self.B), 0, self.zg*self.W*sin_theta, 0])

def main(args=None):
    rclpy.init(args=args)
    node = EulerLagrangeController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
