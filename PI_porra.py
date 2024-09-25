#!/usr/bin/env python3
# Esta linha de shebang indica ao sistema que o script deve ser executado usando o Python 3.

import rclpy  # Importa o módulo principal do ROS2 em Python.
from mpu
from rclpy.node import Node  # Importa a classe base Node, que será usada para criar o nó ROS2.
from std_msgs.msg import Int32  # Importa o tipo de mensagem Int32, que é equivalente ao std_msgs/Int32 no ROS1.

# Definição da classe IntPublisher, que herda da classe Node.
class IntPublisher(Node):

    def __init__(self):
        # Chama o construtor da classe base Node, passando o nome do nó 'int_publisher'.
        super().__init__('int_publisher')

        # Cria um publisher que publicará mensagens do tipo Int32 no tópico 'int_topic'.
        # O segundo parâmetro, 10, define o tamanho da fila de mensagens armazenadas, semelhante ao 'queue_size' no ROS1.
        self.publisher_ = self.create_publisher(Int32, 'int_topic', 10)

        # Define o período do timer em segundos, que irá chamar a função de callback periodicamente.
        timer_period = 1  # segundos
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Inicializa um contador para os números inteiros que serão publicados.
        self.count = 0

    # Esta função é chamada a cada segundo, conforme definido pelo timer.
    def timer_callback(self):
        # Cria uma nova mensagem do tipo Int32.
        msg = Int32()
        # Define o valor da mensagem como o valor atual do contador.
        msg.data = self.count
        # Publica a mensagem no tópico 'int_topic'.
        self.publisher_.publish(msg)
        # Usa o logger do ROS2 para exibir uma mensagem no console.
        self.get_logger().info(f'Publicando: {self.count}')
        # Incrementa o contador.
        self.count += 1

# Função principal que inicializa o nó ROS2 e mantém ele em execução.
def main(args=None):
    # Inicializa o sistema ROS2.
    rclpy.init(args=args)

    # Cria uma instância do nó IntPublisher.
    int_publisher = IntPublisher()

    # Mantém o nó em execução para que ele possa continuar a publicar mensagens.
    rclpy.spin(int_publisher)

    # Quando o nó é interrompido, ele é destruído corretamente.
    int_publisher.destroy_node()

    # Encerra o sistema ROS2.
    rclpy.shutdown()

# Ponto de entrada do script. Se este arquivo for executado diretamente, a função main() será chamada.
if __name__ == '__main__':
    main()
