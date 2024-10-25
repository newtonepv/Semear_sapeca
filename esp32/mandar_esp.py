import serial
import time
import asyncio

# Open the serial port for writing (Replace '/dev/ttyUSB0' with your port)
ser = serial.Serial('/dev/pts/3', 9600)
codigo_para_aumentar_motores = "AT "

async def aumentar_motor(direito:float, esquerdo:float):
    mensagem = codigo_para_aumentar_motores + str(direito) + " " + str(esquerdo)
    #ser.write(mensagem.encode('utf-8'))  
    print(mensagem)
