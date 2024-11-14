import asyncio
import smbus2 as smbus
import time
import math
import numpy as np
from filterpy.kalman import KalmanFilter
from collections import deque

MPU9250_ADDRESS = 0x68
ACCEL_REGISTER = 0x3B
GYRO_REGISTER = 0x43

bus = smbus.SMBus(1)

# Configura o MPU-9250
def setup_mpu9250():
    bus.write_byte_data(MPU9250_ADDRESS, 0x6B, 0x00)  # Wake up MPU-9250
    bus.write_byte_data(MPU9250_ADDRESS, 0x1C, 0x00)  # Configura acelerômetro ±2g
    bus.write_byte_data(MPU9250_ADDRESS, 0x1B, 0x00)  # Configura giroscópio ±250º/s

def ler_2_bytes(addr):
    high = bus.read_byte_data(MPU9250_ADDRESS, addr)
    low = bus.read_byte_data(MPU9250_ADDRESS, addr + 1)
    val = (high << 8) + low
    return -((65535 - val) + 1) if val >= 0x8000 else val

def ler_acceleracao():
    accel_x = ler_2_bytes(ACCEL_REGISTER) / 16384.0
    accel_y = ler_2_bytes(ACCEL_REGISTER + 2) / 16384.0
    accel_z = ler_2_bytes(ACCEL_REGISTER + 4) / 16384.0
    return accel_x, accel_y, accel_z

def ler_gyro():
    gyro_x = ler_2_bytes(GYRO_REGISTER) / 131.0
    gyro_y = ler_2_bytes(GYRO_REGISTER + 2) / 131.0
    gyro_z = ler_2_bytes(GYRO_REGISTER + 4) / 131.0
    return gyro_x, gyro_y, gyro_z

def calcular_pitch_roll(accel_x, accel_y, accel_z):
    pitch = math.atan2(accel_x, math.sqrt(accel_y**2 + accel_z**2)) * 180.0 / math.pi
    roll = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180.0 / math.pi
    return pitch, roll

# Inicialização do sensor
setup_mpu9250()

# Configuração do filtro de Kalman para pitch e roll
kf_pitch = KalmanFilter(dim_x=2, dim_z=1)
kf_roll = KalmanFilter(dim_x=2, dim_z=1)
# Intervalo de tempo entre leituras
dt = 0.1
# Inicializa os parâmetros do filtro (ajuste conforme necessário)
for kf in (kf_pitch, kf_roll):
    kf.x = np.array([0., 0.])        # Estado inicial [ângulo, taxa de mudança]
    kf.F = np.array([[1., dt],     # Matriz de transição (ajuste dt conforme necessário)
                     [0., 1.]])
    kf.H = np.array([[1., 0.]])      # Matriz de observação
    kf.P *= 10.                      # Covariância inicial
    kf.R = np.array([[0.5]])         # Ruído de observação
    kf.Q = np.array([[0.1, 0.],      # Ruído de processo
                     [0., 0.1]])

# Variáveis para a média móvel
WINDOW_SIZE = 10
kalman_roll_window = deque(maxlen=WINDOW_SIZE)

async def angulacao_y():
    accel_x, accel_y, accel_z = ler_acceleracao()
    gyro_x, gyro_y, gyro_z = ler_gyro()
    
    # Cálculo dos ângulos baseados na aceleração
    pitch, roll = calcular_pitch_roll(accel_x, accel_y, accel_z)

    # Filtro de Kalman para pitch
    kf_pitch.predict()
    kf_pitch.update(pitch)
    kalman_pitch = kf_pitch.x[0]

    # Filtro de Kalman para roll
    kf_roll.predict()
    kf_roll.update(roll)
    kalman_roll = kf_roll.x[0]
    
    # Adiciona o novo valor de kalman_roll na janela
    kalman_roll_window.append(kalman_roll)
    
    # Calcula a média móvel
    media_movel_kalman_roll = sum(kalman_roll_window) / len(kalman_roll_window)
    #offset pq ficou torto -4 graus qnd colou
    media_movel_kalman_roll+=3.6
    #print(f"Média Móvel do Kalman Roll: {media_movel_kalman_roll:.5f}")
    if(media_movel_kalman_roll<1 and media_movel_kalman_roll>-1):
        media_movel_kalman_roll = 0

    
    # Exibe a média móvel do Kalman Roll
    #print(f"Média Móvel do Kalman Roll: {media_movel_kalman_roll:.5f}")
    
    return media_movel_kalman_roll
