import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.qos import QoSProfile

class SimpleNavigation(Node):

    def __init__(self):
        super().__init__('simple_navigation')
        # Cria um cliente de ação para se conectar ao servidor 'navigate_to_pose'
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    # Envia o objetivo de navegação para o servidor de ação
    def send_goal(self, pose):
        # Espera até o servidor de ação estar disponível
        self._action_client.wait_for_server()

        # Define o objetivo de navegação
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        # Envia o objetivo de navegação
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    # Callback para lidar com a resposta do servidor de ação
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Objetivo de navegação não foi aceito.')
            return
        self.get_logger().info('Objetivo aceito.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    # Callback para lidar com o resultado da navegação
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Resultado da navegação: {result}')
        rclpy.shutdown()

    # Callback para lidar com o feedback da navegação
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: Distância restante: {feedback.distance_remaining}')

def main(args=None):
    rclpy.init(args=args)

    # Cria uma instância do nó de navegação
    navigation_node = SimpleNavigation()

    # Cria uma pose de destino
    target_pose = PoseStamped()
    target_pose.header.frame_id = 'map'
    target_pose.header.stamp = navigation_node.get_clock().now().to_msg()

    # Definir coordenadas do objetivo (exemplo: 1.0, 1.0, 0.0)
    target_pose.pose.position.x = 1.0
    target_pose.pose.position.y = 1.0
    target_pose.pose.orientation.w = 1.0

    # Enviar o objetivo de navegação
    navigation_node.send_goal(target_pose)

    # Mantém o nó rodando até a navegação completar
    rclpy.spin(navigation_node)

if __name__ == '__main__':
    main()
