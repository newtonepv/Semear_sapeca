#include <AccelStepper.h>

// Definições dos pinos
const int M0d = 13;
const int M1d = 14;
const int M2d = 27;
const int M0e = 23;
const int M1e = 22;
const int M2e = 21;
const int DIR_ESQUERDA = 32;
const int STEP_ESQUERDA = 33;
const int SLEEP_ESQUERDA = 25;
const int RESET_ESQUERDA = 26;
const int DIR_DIREITA = 15;
const int STEP_DIREITA = 4;
const int SLEEP_DIREITA = 18;
const int RESET_DIREITA = 19;

// Criação dos objetos de motor
// Motor direito criado com inversão direta no construtor
AccelStepper stepper_esquerda(1, STEP_ESQUERDA, DIR_ESQUERDA);
AccelStepper stepper_direita(1, STEP_DIREITA, DIR_DIREITA);

// Variáveis para armazenar mensagem serial
String inputString = "";
bool stringComplete = false;

// Função de mapeamento personalizada para 0-100 para -499 a 499
long mapearValor(long x, long in_min, long in_max, long out_min, long out_max) {
  // Primeiro garante que o valor de entrada esteja dentro do intervalo
  x = constrain(x, in_min, in_max);
  
  // Mapeamento que permite movimento em ambas as direções
  if (x == 50) return 0;  // Ponto neutro
  else if (x < 50) {
    // Mapeamento para trás (0-49 → 0 a -499)
    return map(x, 0, 49, -499, 0);
  } else {
    // Mapeamento para frente (51-100 → 0 a 499)
    return map(x, 51, 100, 0, 499);
  }
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  
  // Configuração dos pinos de microstepping para motor esquerdo
  pinMode(M0e, OUTPUT);
  pinMode(M1e, OUTPUT);
  pinMode(M2e, OUTPUT);
  
  // Configuração dos pinos de microstepping para motor direito
  pinMode(M0d, OUTPUT);
  pinMode(M1d, OUTPUT);
  pinMode(M2d, OUTPUT);
  
  // Configuração dos pinos de SLEEP e RESET
  pinMode(SLEEP_ESQUERDA, OUTPUT);
  pinMode(RESET_ESQUERDA, OUTPUT);
  pinMode(SLEEP_DIREITA, OUTPUT);
  pinMode(RESET_DIREITA, OUTPUT);
  
  // Ativar motores (SLEEP LOW = Ativo, RESET HIGH = Funcionando)
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(RESET_ESQUERDA, HIGH);
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(RESET_DIREITA, HIGH);
  
  // Configurar microstepping (full step para máxima velocidade)
  // Motor Esquerdo
  digitalWrite(M0e, LOW);
  digitalWrite(M1e, LOW);
  digitalWrite(M2e, LOW);
  
  // Motor Direito
  digitalWrite(M0d, LOW);
  digitalWrite(M1d, LOW);
  digitalWrite(M2d, LOW);
  
  // Configurações padrão dos motores
  stepper_esquerda.setMaxSpeed(400);
  stepper_esquerda.setAcceleration(5000);
  
  stepper_direita.setMaxSpeed(400);
  stepper_direita.setAcceleration(5000);
}

void loop() {
  if (stringComplete) {
    processaComando(inputString);
    
    inputString = "";
    stringComplete = false;
  }
  
  stepper_esquerda.run();
  stepper_direita.run();
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    
    inputString += inChar;
    
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void processaComando(String comando) {
  int indiceMe = comando.indexOf("Me ");
  int indiceMd = comando.indexOf("Md ");
  
  if (indiceMe != -1 && indiceMd != -1) {
    int valorEsquerda = comando.substring(indiceMe + 3, indiceMd).toInt();
    int valorDireita = comando.substring(indiceMd + 3).toInt();
    

    bool direcaoE = false;
    if(valorEsquerda<0){
      valorEsquerda *= -1;
      direcaoE=true;
    }

    bool direcaoD = false;
    if(valorDireita<0){
      valorDireita *= -1;
      direcaoD=true;
    }


    long passosEsquerda = mapearValor(valorEsquerda, 0, 360, 0, 499);
    long passosDireita = mapearValor(valorDireita, 0, 360, 0, 499);
    

    alterarVelocidade(passosEsquerda, passosDireita);
    
  }
}

void moverMotores(long passos_esquerda, long passos_direita) {
  stepper_esquerda.moveTo(passos_esquerda);
  stepper_direita.moveTo(passos_direita);
}

void alterarVelocidade(int velocidade_esquerda, int velocidade_direita) {
  stepper_esquerda.setSpeed(velocidade_esquerda);
  stepper_direita.setSpeed(velocidade_direita); 
}
