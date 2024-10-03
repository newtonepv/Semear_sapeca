import smbus
import time

# Endereço I2C do MPU-9250 (com MPU-6500 embutido)
MPU9250_ADDRESS = 0x68

# Registradores do acelerômetro (partes altas)
ACCEL_XOUT_H = 0x3B

# Registradores do giroscópio (partes altas)
GYRO_XOUT_H = 0x43

# Inicializa o barramento I2C
bus = smbus.SMBus(1)  # A maioria das Raspberry Pi usa o bus 1

# Função para configurar o MPU-9250
def setup_mpu9250():
    # Despertar o MPU-9250 (sai do modo de sono)
    bus.write_byte_data(MPU9250_ADDRESS, 0x6B, 0x00)
    # Configurar a escala do acelerômetro para +/- 2g
    bus.write_byte_data(MPU9250_ADDRESS, 0x1C, 0x00)
    # Configurar a escala do giroscópio para +/- 250 graus/seg
    bus.write_byte_data(MPU9250_ADDRESS, 0x1B, 0x00)

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

# Configura o MPU-9250
setup_mpu9250()

# Loop principal para ler os dados continuamente
try:
    while True:
        # Leitura dos dados do acelerômetro
        accel_x, accel_y, accel_z = read_accel_data()
        print(f'Acelerômetro -> X: {accel_x:.2f} g, Y: {accel_y:.2f} g, Z: {accel_z:.2f} g')

        # Leitura dos dados do giroscópio
        gyro_x, gyro_y, gyro_z = read_gyro_data()
        print(f'Giroscópio -> X: {gyro_x:.2f} °/s, Y: {gyro_y:.2f} °/s, Z: {gyro_z:.2f} °/s')
        
        # Aguardar meio segundo antes de ler novamente
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Leitura interrompida")
