#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

# Função de callback que é chamada sempre que uma nova mensagem é recebida no tópico inscrito.
def int_callback(data):
    # Exibe no log do ROS a mensagem recebida. O conteúdo da mensagem é acessado por 'data.data'.
    rospy.loginfo(f"Recebido: {data.data}")

# Função principal que configura o nó subscriber.
def int_subscriber():
    # Inicializa o nó ROS chamado 'int_subscriber'. O parâmetro 'anonymous=True' cria um nome único para o nó,
    # evitando conflitos caso mais de um nó com o mesmo nome seja executado.
    rospy.init_node('int_subscriber', anonymous=True)
    
    # Configura o subscriber para o tópico 'int_topic'. Ele escutará mensagens do tipo Int32.
    # A função 'int_callback' é registrada como callback e será chamada sempre que uma nova mensagem for recebida.
    rospy.Subscriber('int_topic', Int32, int_callback)
    
    # Mantém o nó ativo e processando callbacks até que o ROS seja desligado.
    rospy.spin()

# Bloco principal que executa a função int_subscriber, com tratamento de exceção para interrupção ROS.
if __name__ == '__main__':
    int_subscriber()