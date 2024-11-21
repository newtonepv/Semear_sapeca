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

// Limites de velocidade (em microssegundos)
const int VELOCIDADE_MIN = 1;   // Velocidade máxima (menor delay)
const int VELOCIDADE_MAX = 300;  // Velocidade mínima (maior delay)

// Variáveis para armazenar o estado atual dos motores
struct MotorState {
  int valor;        // Valor original (-499 a 499)
  int velocidade;   // Velocidade mapeada
  bool direcao;     // Direção atual
};

MotorState motorDireito = {0, 0, true};
MotorState motorEsquerdo = {0, 0, true};

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(115200);
  inputString.reserve(200);
  pinMode(2, OUTPUT);
  // Configuração dos pinos dos motores
  pinMode(STEP_ESQUERDA, OUTPUT);
  pinMode(DIR_ESQUERDA, OUTPUT);
  pinMode(SLEEP_ESQUERDA, OUTPUT);
  pinMode(RESET_ESQUERDA, OUTPUT);

  pinMode(STEP_DIREITA, OUTPUT);
  pinMode(DIR_DIREITA, OUTPUT);
  pinMode(SLEEP_DIREITA, OUTPUT);
  pinMode(RESET_DIREITA, OUTPUT);

  // Ativa os drivers
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(RESET_ESQUERDA, HIGH);
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(RESET_DIREITA, HIGH);
  
  // Configuração do microstepping
  pinMode(M0d, OUTPUT);
  pinMode(M1d, OUTPUT);
  pinMode(M2d, OUTPUT);
  pinMode(M0e, OUTPUT);
  pinMode(M1e, OUTPUT);
  pinMode(M2e, OUTPUT);

  // Define modo de microstepping
  digitalWrite(M0d, LOW);
  digitalWrite(M1d, LOW);
  digitalWrite(M2d, HIGH);

  digitalWrite(M0e, LOW);
  digitalWrite(M1e, LOW);
  digitalWrite(M2e, HIGH);

  Serial.println("ESP32 pronto!");
  Serial.println("Use o formato: Me {valor1} Md {valor2}");
  Serial.println("Valores entre -499 e 499");
}

void loop() {

  // Verifica se há novos comandos
  checkSerialCommand();
  
  // Move os motores com base nos estados atuais
  moveMotors();
  
}

void checkSerialCommand() {
  while (Serial.available()) {
    //digitalWrite(2, HIGH);
    //Serial.println("sexo");
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      processCommand(inputString);
      inputString = "";
    } else {
      inputString += inChar;
    }
  }
}

// Função para limitar valor entre -499 e 499
int limitarValor(int valor) {
  if (valor > 499) return 499;
  if (valor < -499) return -499;
  return valor;
}

// Função para mapear valor de velocidade
int mapearVelocidade(int valor) {
  // Para valor = 0, retorna 0 (motor parado)
  if (valor == 0) return 0;
  
  // Mapeia o valor absoluto de 1-499 para VELOCIDADE_MAX até VELOCIDADE_MIN
  int valorAbs = abs(valor);
  return map(valorAbs, 1, 499, VELOCIDADE_MAX, VELOCIDADE_MIN);
}

void processCommand(String command) {
  command.trim();
  //command = [Me 8 Md 4]
  //command[0] = Me
  //command[5] = Md
  int mdIndex = command.indexOf("Md");
  int meIndex = command.indexOf("Me");
  
  if (mdIndex != -1 && meIndex != -1) {
    // Extrai os valores entre "Md" e "Me"
    String valorEsquerda = command.substring(meIndex + 2, mdIndex);
    String valorDireita = command.substring(mdIndex + 2);
    


    // Atualiza estados dos motores
    motorDireito.valor = limitarValor(valorDireita.toInt());// Função para limitar valor entre -499 e 499
    motorDireito.velocidade = mapearVelocidade(motorDireito.valor);
    motorDireito.direcao = (motorDireito.valor >= 0);

    motorEsquerdo.valor = limitarValor(valorEsquerda.toInt());
    motorEsquerdo.velocidade = mapearVelocidade(motorEsquerdo.valor);
    motorEsquerdo.direcao = (motorEsquerdo.valor >= 0);

    // Define as direções dos motores
    digitalWrite(DIR_DIREITA, motorDireito.direcao ? HIGH : LOW);
    digitalWrite(DIR_ESQUERDA, motorEsquerdo.direcao ? LOW : HIGH);
    
    /* Feedback
    Serial.print("Motor Direita: ");
    Serial.print(motorDireito.valor);
    Serial.print(" (delay: ");
    Serial.print(motorDireito.velocidade);
    Serial.print("us) | Motor Esquerda: ");
    Serial.print(motorEsquerdo.valor);
    Serial.print(" (delay: ");
    Serial.print(motorEsquerdo.velocidade);
    Serial.println("us)");*/
  }
}

void moveMotors() {
  // Move motor direito se tiver velocidade
  if (motorDireito.velocidade > 0) {
    digitalWrite(STEP_DIREITA, HIGH);
    delayMicroseconds(motorDireito.velocidade);
    Serial.println(motorDireito.velocidade);
    Serial.println(motorDireito.direcao);
    digitalWrite(STEP_DIREITA, LOW);
    delayMicroseconds(motorDireito.velocidade);
  }
  
  // Move motor esquerdo se tiver velocidade
  if (motorEsquerdo.velocidade > 0) {
    digitalWrite(STEP_ESQUERDA, HIGH);
    delayMicroseconds(motorEsquerdo.velocidade);
    digitalWrite(STEP_ESQUERDA, LOW);
    delayMicroseconds(motorEsquerdo.velocidade);
  }
}
