import smbus
import time
import math

# Endereço I2C do MPU-9250 (com MPU-6500 embutido)
MPU9250_ADDRESS = 0x68

# Registradores do acelerômetro (partes altas)
ACCEL_REGISTER = 0x3B

# Registradores do giroscópio (partes altas)
GYRO_REGISTER = 0x43

# Registradores do magnetômetro
MAG_REGISTER = 0x03  # O primeiro registrador do magnetômetro

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

# Função para ler um valor de 16 bits (2 registradores) do MPU-9250
def ler_2_bytes(addr):
    high = bus.read_byte_data(MPU9250_ADDRESS, addr)
    low = bus.read_byte_data(MPU9250_ADDRESS, addr + 1)
    val = (high << 8) + low
    if val >= 0x8000:
        return -((65535 - val) + 1)
    else:
        return val

# Função para ler os dados do acelerômetro
def ler_acceleracao_angular():
    accel_x = ler_2_bytes(ACCEL_REGISTER)
    accel_y = ler_2_bytes(ACCEL_REGISTER + 2)
    accel_z = ler_2_bytes(ACCEL_REGISTER + 4)
    
    # Escala de aceleração é +/- 2g, então convertemos os valores para g's
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0

    return accel_x, accel_y, accel_z

# Função para ler os dados do giroscópio
def read_gyro_data():
    gyro_x = ler_2_bytes(GYRO_REGISTER)
    gyro_y = ler_2_bytes(GYRO_REGISTER + 2)
    gyro_z = ler_2_bytes(GYRO_REGISTER + 4)

    # Escala de giroscópio é +/- 250 graus/seg, converte para graus/segundo
    gyro_x = gyro_x / 131.0
    gyro_y = gyro_y / 131.0
    gyro_z = gyro_z / 131.0

    return gyro_x, gyro_y, gyro_z


# Função para calcular os ângulos de inclinação a partir do acelerômetro
def calcular_pitch_roll(accel_x, accel_y, accel_z):
    pitch = math.atan2(accel_y, math.sqrt(accel_x**2 + accel_z**2)) * 180.0 / math.pi
    roll = math.atan2(accel_x, accel_z) * 180.0 / math.pi
    return pitch, roll


# Configura o MPU-9250
setup_mpu9250()

pitch = 0.0
roll = 0.0

# Tempo inicial para cálculo das integrais
last_time = time.time()

# Loop principal para ler os dados continuamente
try:
    # Leitura dos dados do acelerômetro
    ang_accel_x, ang_accel_y, ang_accel_z = ler_acceleracao_angular()
    
    # Leitura dos dados do giroscópio
    gyro_x, gyro_y, gyro_z = read_gyro_data()

    # Cálculo do tempo decorrido desde a última leitura
    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    # Calcula os ângulos de inclinação a partir dos dados do acelerômetro
    accel_pitch, accel_roll = calcular_pitch_roll(ang_accel_x, ang_accel_y, ang_accel_z)

    # Integra os dados do giroscópio para obter a variação dos ângulos
    gyro_pitch_rate = gyro_x  # Taxa de rotação ao redor do eixo X (pitch)
    gyro_roll_rate = gyro_y   # Taxa de rotação ao redor do eixo Y (roll)

    # Atualiza os ângulos com os dados do giroscópio
    pitch += gyro_pitch_rate * dt
    roll += gyro_roll_rate * dt

    # Aplica o filtro complementar para combinar os dados do acelerômetro e giroscópio
    pitch = ALPHA * pitch + (1 - ALPHA) * accel_pitch
    roll = ALPHA * roll + (1 - ALPHA) * accel_roll
    
    while True:
        # Leitura dos dados do acelerômetro
        ang_accel_x, ang_accel_y, ang_accel_z = ler_acceleracao_angular()
        
        # Leitura dos dados do giroscópio
        gyro_x, gyro_y, gyro_z = read_gyro_data()

        # Cálculo do tempo decorrido desde a última leitura
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # Calcula os ângulos de inclinação a partir dos dados do acelerômetro
        accel_pitch, accel_roll = calcular_pitch_roll(ang_accel_x, ang_accel_y, ang_accel_z)

        # Integra os dados do giroscópio para obter a variação dos ângulos
        gyro_pitch_rate = gyro_x  # Taxa de rotação ao redor do eixo X (pitch)
        gyro_roll_rate = gyro_y   # Taxa de rotação ao redor do eixo Y (roll)

        # Atualiza os ângulos com os dados do giroscópio
        pitch += gyro_pitch_rate * dt
        roll += gyro_roll_rate * dt

        # Aplica o filtro complementar para combinar os dados do acelerômetro e giroscópio
        pitch = ALPHA * pitch + (1 - ALPHA) * accel_pitch
        roll = ALPHA * roll + (1 - ALPHA) * accel_roll



        # Exibe os ângulos calculados
        print(f'Pitch: {pitch:.2f}°, Roll: {roll:.2f}°')
        
        

        # Aguardar antes de ler novamente (é bom que seja pouco para integrar a velocidade direitinho)
        time.sleep(0.0001)

except KeyboardInterrupt:
    print("Leitura interrompida")
