#esquematico: https://lucid.app/lucidchart/62260741-46a0-4a66-802a-28311fa6e7e1/edit?invitationId=inv_97b34dea-c6b7-4c33-a997-008dc080b8b4&page=zok3yUf69dyn#
from pid.pid import Pid
import time

pid_motor = Pid(3, 1, 0)


while(True):
    start = time.perf_counter()
    i=100000
    while(i):
        i=i-1
    end = time.perf_counter()
    print(pid_motor.update(1,2,start-end))
