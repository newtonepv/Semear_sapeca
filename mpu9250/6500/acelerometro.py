import smbus
import time

#Endereço I2C do MPU-9250 no barramento I2C da Raspberry Pi. É o endereço usado para comunicação entre a Pi e o sensor.
MPU9250_ADDRESS = 0x68
# Endereço de um dos registradores do acelerômetro, que contém o valor da aceleração no eixo X.
ACCEL_XOUT_H = 0x3B  

# Inicializa o barramento I2C
bus = smbus.SMBus(1)  # A maioria das RPi usa o bus 1

# Função para configurar o MPU-9250
def setup_mpu9250():
    # Despertar o MPU-9250 (sai do modo de sono) --> 0x00 coloca no modo padrão ou mínimo de funcionamento, que neste caso é fazer acordar(0x6B controla a energia)
    bus.write_byte_data(MPU9250_ADDRESS, 0x6B, 0x00)
    # Configurar a escala do acelerômetro para +/- 2g (valor padrão da leitura do acelerometro) --> 0x1C controla a escala do acelerometro
    bus.write_byte_data(MPU9250_ADDRESS, 0x1C, 0x00)

# Função para ler um valor de 16 bits (2 registradores) do MPU-9250 a partir do endereço de leitura da aceleração em x
def read_word_2c(addr):
    #Byte alto e baixo, cada um com 1 byte(16 bits no total)
    high = bus.read_byte_data(MPU9250_ADDRESS, addr)
    low = bus.read_byte_data(MPU9250_ADDRESS, addr + 1)
    val = (high << 8) + low #Cria o valor completo, com 16 bytes, mas antes da um shift pra esquerda de 1 byte, tamanho do registrador do byte alto 
    #Verifica se o valor é negativo (acima de 0x8000 --> complemento de dois).Se for negativo, a função retorna o valor negativo correspondente.
    if val >= 0x8000:   
        return -((65535 - val) + 1)
    else:
        return val

# Função para ler os dados do acelerômetro
def read_accel_data():
    accel_x = read_word_2c(ACCEL_XOUT_H) #Lê o valor de 16 bits do eixo X a partir do registrador ACCEL_XOUT_H(setado no começo com o endereço do eixo X).
    accel_y = read_word_2c(ACCEL_XOUT_H + 2)#Lê o valor de 16 bits do eixo Y, com o endereço deslocado por 2 registradores(endereço de X + tamanho de X).
    accel_z = read_word_2c(ACCEL_XOUT_H + 4)#Lê o valor de 16 bits do eixo Y, com o endereço deslocado por 4 registradores(endereço de X + tamanho de X+Y).
    
    #Converte o valor lido para a unidade de "g" (gravidade), já que o sensor está configurado para uma escala de +/- 2g, onde 1g = 16384.
    accel_x = accel_x / 16384.0
    accel_y = accel_y / 16384.0
    accel_z = accel_z / 16384.0

    return accel_x, accel_y, accel_z

# Configura o MPU-9250
setup_mpu9250()

# Loop principal para ler os dados continuamente
try:
    while True:
        accel_x, accel_y, accel_z = read_accel_data()
        print(f'Acelerômetro X: {accel_x:.2f} g, Y: {accel_y:.2f} g, Z: {accel_z:.2f} g')
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Leitura interrompida")
