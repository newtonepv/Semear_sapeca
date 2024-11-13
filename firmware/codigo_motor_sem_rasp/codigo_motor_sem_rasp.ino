/* Montagem da PCB:
  // DIR esquerda 32
  // STEP esquerda 33
  // SLEEP esquerda 25
  // RESET esquerda 26
  // DIR direita 15
  // STEP direita 4
  // SLEEP direita 18
  // RESET direita 19
  // m0 34
  // m1 35
  // m2 32
*/

const int DIR_ESQUERDA = 32;       // Pino para direção do motor esquerdo
const int STEP_ESQUERDA = 33;      // Pino para passo do motor esquerdo
const int SLEEP_ESQUERDA = 25;     // Pino para Sleep do motor esquerdo
const int RESET_ESQUERDA = 26;     // Pino para Reset do motor esquerdo

const int DIR_DIREITA = 15;        // Pino para direção do motor direito
const int STEP_DIREITA = 4;        // Pino para passo do motor direito
const int SLEEP_DIREITA = 18;      // Pino para Sleep do motor direito
const int RESET_DIREITA = 19;      // Pino para Reset do motor direito

const int steps_per_rev = 2000;     // Passos por revolução do motor
const int velocidadeConstante = 1000; // Tempo fixo entre passos para manter a velocidade constante

void setup() {
  // Configuração dos pinos dos motores como saída
  pinMode(STEP_ESQUERDA, OUTPUT);
  pinMode(DIR_ESQUERDA, OUTPUT);
  pinMode(SLEEP_ESQUERDA, OUTPUT);
  pinMode(RESET_ESQUERDA, OUTPUT);

  pinMode(STEP_DIREITA, OUTPUT);
  pinMode(DIR_DIREITA, OUTPUT);
  pinMode(SLEEP_DIREITA, OUTPUT);
  pinMode(RESET_DIREITA, OUTPUT);

  // Ativa os drivers dos motores
  digitalWrite(SLEEP_ESQUERDA, HIGH);
  digitalWrite(RESET_ESQUERDA, HIGH);
  digitalWrite(SLEEP_DIREITA, HIGH);
  digitalWrite(RESET_DIREITA, HIGH);

  // Define a direção inicial dos motores (exemplo: para frente)
  digitalWrite(DIR_ESQUERDA, HIGH);
  digitalWrite(DIR_DIREITA, HIGH);
}

void loop() {
  // Alterna a direção dos motores a cada 5 segundos
  static unsigned long lastChangeTime = 0;  // Armazena o tempo da última mudança
  static bool direction = true;             // Direção inicial (true = HIGH, false = LOW)
  
  // Verifica se já passaram 5 segundos para mudar a direção
  if (millis() - lastChangeTime >= 5000) {
    direction = !direction;  // Alterna a direção
    digitalWrite(DIR_ESQUERDA, direction ? HIGH : LOW);
    digitalWrite(DIR_DIREITA, direction ? HIGH : LOW);
    lastChangeTime = millis();  // Atualiza o tempo da última mudança
  }

  // Movimenta os motores esquerdo e direito simultaneamente
  for (int i = 0; i < steps_per_rev; i++) {
    digitalWrite(STEP_ESQUERDA, HIGH);
    digitalWrite(STEP_DIREITA, HIGH);
    delayMicroseconds(velocidadeConstante);
    
    digitalWrite(STEP_ESQUERDA, LOW);
    digitalWrite(STEP_DIREITA, LOW);
    delayMicroseconds(velocidadeConstante);
  }

  delay(500); // Pausa entre rotações
}

