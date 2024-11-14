import serial
import time
import asyncio

codigo_motor_esquerdo = "Me "
codigo_motor_direito = " Md "

async def motores(direito:float, esquerdo:float, ser:serial.Serial):
    mensagem = codigo_motor_esquerdo + str(int(esquerdo)) + codigo_motor_direito + str(int(direito)) +"\n"
    ser.write(mensagem.encode('utf-8'))  
    #print(mensagem)
