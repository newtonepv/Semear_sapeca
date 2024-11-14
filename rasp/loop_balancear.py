import asyncio
import time
from pid.pid import Pid
from mpu9250 import acelerometro
from esp32 import mandar_esp
import serial

angulo_setpoint= [0]
async def pid_loop(pid_angulo: Pid, ser :serial.Serial):
    # Tempo inicial para o controle de ângulo
    start_angulo = time.perf_counter()
    
    while True:
        # Lê o valor da angulação
        angulacao = await acelerometro.angulacao_y()
        
        # Calcula o tempo decorrido desde a última atualização
        fim_angulo = time.perf_counter()
        delta = fim_angulo - start_angulo
        
        # Atualiza o PID usando o ângulo desejado como setpoint (por exemplo, 0° para balanço)
          # Mantemos o ângulo em 0 para balanceamento
        motores = pid_angulo.update(angulacao, angulo_setpoint[0], delta)
        
        # Atualiza o tempo inicial para o próximo ciclo
        start_angulo = fim_angulo

        if(motores>=499):
            motores=499
        if(motores<=-499):
            motores=-499
        await mandar_esp.motores(motores, motores, ser)
        print(motores)  # Deve imprimir o valor dos motores
        
        # Envia o valor calculado para os motores no ESP32
        # await mandar_esp.aumentar_motor(motores, motores)
        
        # Aguardar para a próxi
        # ma atualização do loop
        await asyncio.sleep(0.001)

# Função principal que configura e executa o loop PID
'''async def main():
    pid_angulo = Pid(kp=2.35, ki=0.1, kd=0)  # Configura os parâmetros do PID conforme necessário
    await pid_loop(pid_angulo)

# Executa a função main no contexto do asyncio
asyncio.run(main())
'''