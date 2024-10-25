import asyncio
import time
from pid.pid import Pid
from mpu9250 import acelerometro
from esp32 import mandar_esp

async def pid_loop(pid_angulo:Pid, pid_velocidade:Pid):
    start_velocidade = time.perf_counter()
    start_angulo = time.perf_counter()
    
    while(True):
        print("aiai")

        angulacao = await acelerometro.angulacao_y()
        velocidade = await acelerometro.angulacao_y()

        fim_velocidade = time.perf_counter()
        delta = start_velocidade-fim_velocidade
        angulo_setpoint = pid_velocidade.update(velocidade,2,delta)#a gente quer 2 m/s
        start_velocidade = time.perf_counter()

        fim_angulo = time.perf_counter()
        delta = start_angulo-fim_angulo
        motores = pid_angulo.update(angulacao,angulo_setpoint,delta)
        start_angulo = time.perf_counter()

        await asyncio.sleep(0.1)
        await mandar_esp.aumentar_motor(motores,motores)
        