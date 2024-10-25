#esquematico: https://lucid.app/lucidchart/62260741-46a0-4a66-802a-28311fa6e7e1/edit?invitationId=inv_97b34dea-c6b7-4c33-a997-008dc080b8b4&page=zok3yUf69dyn#

import time
import asyncio
import loop_detectar_rotinas
import loop_balancear
from pid.pid import Pid

time.sleep(2)#Por causa de abrir a porta da esp32

async def main():
    pid_angulo = Pid(3, 1, 0)
    pid_velocidade = Pid(3,1,0)

    pid_balancear_task=asyncio.create_task(loop_balancear.pid_loop(pid_angulo,pid_velocidade))
    detectar_task=asyncio.create_task(loop_detectar_rotinas.rodar_loop())

    await asyncio.gather(pid_balancear_task, detectar_task)

asyncio.run(main())
