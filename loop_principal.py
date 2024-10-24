#esquematico: https://lucid.app/lucidchart/62260741-46a0-4a66-802a-28311fa6e7e1/edit?invitationId=inv_97b34dea-c6b7-4c33-a997-008dc080b8b4&page=zok3yUf69dyn#
from pid.pid import Pid
import time
import asyncio
import mpu9250.acelerometro
import detector_certo

async def pid_loop(pid_motor:Pid):
    while(True):
        start = time.perf_counter()

        await asyncio.sleep(0.1)
        sensor = await mpu9250.acelerometro.angulacao_y()
        end = time.perf_counter()
        print(pid_motor.update(sensor,2,start-end))

async def main():
    pid_motor = Pid(3, 1, 0)

    pid_task=asyncio.create_task(pid_loop(pid_motor))
    detect_task=asyncio.create_task(detector_certo.detect())

    await asyncio.gather(pid_task, detect_task)

asyncio.run(main())
