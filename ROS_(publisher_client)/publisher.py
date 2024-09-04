#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

# Função principal que será executada para publicar inteiros
def int_publisher():
    # Inicializa o nó ROS chamado 'int_publisher'. O parâmetro 'anonymous=True' cria um nome único para o nó,
    # evitando conflitos caso mais de um nó com o mesmo nome seja executado.
    rospy.init_node('int_publisher', anonymous=True)
    
    # Cria um objeto Publisher que publicará mensagens do tipo Int32 no tópico 'int_topic'.
    # O parâmetro 'queue_size=10' define o tamanho da fila de mensagens armazenadas.
    pub = rospy.Publisher('int_topic', Int32, queue_size=10)
    
    # Define a taxa de publicação para 1 Hz, ou seja, 1 mensagem por segundo.
    rate = rospy.Rate(1)

    # Variável para contar os números inteiros que serão publicados.
    count = 0
    
    # Loop principal que continua até que o ROS seja desligado.
    while not rospy.is_shutdown():
        # Registra no log a mensagem que está sendo publicada.
        rospy.loginfo(f"Publicando: {count}")
        
        # Publica o valor atual de 'count' no tópico 'int_topic'.
        pub.publish(count)
        
        # Incrementa o contador para a próxima publicação.
        count += 1
        
        # Aguarda o tempo necessário para manter a taxa de publicação definida (1 Hz).
        rate.sleep()

# Bloco principal que executa a função int_publisher, com tratamento de exceção para interrupção ROS.
if __name__ == '__main__':
    try:
        int_publisher()
    except rospy.ROSInterruptException:
        # Caso o ROS seja interrompido (ex: com Ctrl+C), o programa lidará com isso sem erros.
        pass