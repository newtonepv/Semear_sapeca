#esquematico: https://lucid.app/lucidchart/62260741-46a0-4a66-802a-28311fa6e7e1/edit?invitationId=inv_97b34dea-c6b7-4c33-a997-008dc080b8b4&page=zok3yUf69dyn#
from esp32.mandar_esp import motores
import time
import asyncio
import mpu9250.acelerometro
import loop_detectar_rotinas
import loop_balancear
from pid.pid import Pid
import serial

time.sleep(1)#Por causa de abrir a porta da esp32

async def main():
    ser = serial.Serial('/dev/ttyUSB0', 115200)
    pid_angulo = Pid(kp=30, ki=0.01, kd=1)
    pid_balancear_task=asyncio.create_task(loop_balancear.pid_loop(pid_angulo,ser))
    
    angulo_setpoint=loop_balancear.angulo_setpoint

    detectar_task=asyncio.create_task(loop_detectar_rotinas.loop_detector(angulo_setpoint))
    try:
        await asyncio.gather(pid_balancear_task, detectar_task)
    except asyncio.CancelledError:
        await motores(0, 0, ser)


asyncio.run(main())
