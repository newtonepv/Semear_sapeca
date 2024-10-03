import smbus
import time
import math

# Endereço I2C do MPU-9250 (com MPU-6500 embutido)
MPU9250_ADDRESS = 0x68
MAG_ADDRESS = 0x0C  # Endereço do magnetômetro AK8963

# Registradores do acelerômetro (partes altas)
ACCEL_XOUT_H = 0x3B

# Registradores do giroscópio (partes altas)
GYRO_XOUT_H = 0x43

# Registradores do magnetômetro
MAG_XOUT_H = 0x03  # O primeiro registrador do magnetômetro

# Inicializa o barramento I2C
bus = smbus.SMBus(1)  # A maioria das Raspberry Pi usa o bus 1

# Filtro complementar constante (ajustável)
ALPHA = 0.98

# Função para configurar o MPU-9250
def setup_mpu9250():
    # Despertar o MPU-9250 (sai do modo de sono)
    bus.write_byte_data(MPU9250_ADDRESS, 0x6B, 0x00)
    # Configurar a escala do acelerômetro para +/- 2g
    bus.write_byte_data(MPU9250_ADDRESS, 0x1C, 0x00)
    # Configurar a escala do giroscópio para +/- 250 graus/seg
    bus.write_byte_data(MPU9250_ADDRESS, 0x1B, 0x00)
    # Configurar o magnetômetro
    bus.write_byte_data(MAG_ADDRESS, 0x0A, 0x16)  # Configura o modo de operação do magnetômetro

# Função para ler um valor de 16 bits (2 registradores) do MPU-9250
def read_word_2c(addr):
    high = bus.read_byte_data(MPU9250_ADDRESS, addr)
    low = bus.read_byte_data(MPU9250_ADDRESS, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

# Função para ler os dados do acelerômetro
def read_accel_data():
    accel_x = read_word_2c(ACCEL_XOUT_H)
    accel_y = read_word_2c(ACCEL_XOUT_H + 2)
    accel_z = read_word_2c(ACCEL_XOUT_H + 4)
    
    # Escala de aceleração é +/- 2g, então convertemos os valores para g's
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0

    return accel_x, accel_y, accel_z

# Função para ler os dados do giroscópio
def read_gyro_data():
    gyro_x = read_word_2c(GYRO_XOUT_H)
    gyro_y = read_word_2c(GYRO_XOUT_H + 2)
    gyro_z = read_word_2c(GYRO_XOUT_H + 4)

    # Escala de giroscópio é +/- 250 graus/seg, converte para graus/segundo
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0

    return gyro_x, gyro_y, gyro_z

# Função para ler os dados do magnetômetro
def read_mag_data():
    # Lê os dados do magnetômetro
    mag_x = read_word_2c(MAG_XOUT_H)
    mag_y = read_word_2c(MAG_XOUT_H + 2)
    mag_z = read_word_2c(MAG_XOUT_H + 4)

    return mag_x, mag_y, mag_z

# Função para calcular os ângulos de inclinação a partir do acelerômetro
def calculate_accel_angles(accel_x, accel_y, accel_z):
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180.0 / math.pi
    roll = math.atan2(accel_x, accel_z) * 180.0 / math.pi
    return pitch, roll

# Função para calcular Yaw usando o magnetômetro
def calculate_yaw(mag_x, mag_y):
    # O yaw pode ser calculado usando a função atan2
    yaw = math.atan2(mag_y, mag_x) * 180.0 / math.pi
    if yaw < 0:
        yaw += 360
    return yaw

# Configura o MPU-9250
setup_mpu9250()

# Inicialização das variáveis de ângulo
pitch = 0.0
roll = 0.0
yaw = 0.0

# Tempo inicial para cálculo da diferença de tempo
last_time = time.time()

# Loop principal para ler os dados continuamente
try:
    while True:
        # Leitura dos dados do acelerômetro
        accel_x, accel_y, accel_z = read_accel_data()
        
        # Leitura dos dados do giroscópio
        gyro_x, gyro_y, gyro_z = read_gyro_data()

        # Leitura dos dados do magnetômetro
        mag_x, mag_y, mag_z = read_mag_data()

        # Cálculo do tempo decorrido desde a última leitura
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Calcula os ângulos de inclinação a partir dos dados do acelerômetro
        accel_pitch, accel_roll = calculate_accel_angles(accel_x, accel_y, accel_z)

        # Integra os dados do giroscópio para obter a variação dos ângulos
        gyro_pitch_rate = gyro_x  # Taxa de rotação ao redor do eixo X (pitch)
        gyro_roll_rate = gyro_y   # Taxa de rotação ao redor do eixo Y (roll)
        gyro_yaw_rate = gyro_z    # Taxa de rotação ao redor do eixo Z (yaw)

        # Atualiza os ângulos com os dados do giroscópio
        pitch += gyro_pitch_rate * dt
        roll += gyro_roll_rate * dt
        yaw += gyro_yaw_rate * dt

        # Aplica o filtro complementar para combinar os dados do acelerômetro e giroscópio
        pitch = ALPHA * pitch + (1 - ALPHA) * accel_pitch
        roll = ALPHA * roll + (1 - ALPHA) * accel_roll

        # Corrige o yaw usando o magnetômetro
        yaw = calculate_yaw(mag_x, mag_y)

        # Exibe os ângulos calculados
        print(f'Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°')

        # Aguardar meio segundo antes de ler novamente
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Leitura interrompida")
