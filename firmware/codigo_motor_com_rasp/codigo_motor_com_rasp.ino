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

int steps_per_rev = 200;           // Passos por revolução do motor

int velGrossaEsquerda, velGrossaDireita;    // Velocidades recebidas para os motores
int velTratadaEsquerda, velTratadaDireita;  // Velocidades tratadas para os motores

// Função para tratar valores de velocidade
void tratarValor(int velGrossa, int &velTratada) {
  velTratada = map(abs(velGrossa), 0, 100, 600, 1500);  // Ajuste para receber valores entre -100 e 100 e transformar em um valor entre 600 e 1500.
}

void setup() {
  Serial.begin(115200);

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

  // Inicia com velocidades zero
  velGrossaEsquerda = velGrossaDireita = 0;
  tratarValor(velGrossaEsquerda, velTratadaEsquerda);
  tratarValor(velGrossaDireita, velTratadaDireita);
}

void loop() {
  // Verifica se há dados disponíveis na porta serial
  if (Serial.available() > 0) {
    // Lê a string até o caractere de nova linha
    String data = Serial.readStringUntil('\n');

    // Extrai o valor para o motor esquerdo (Me)
    int indexMe = data.indexOf("Me ");
    if (indexMe != -1) {
      String velEsquerdaStr = data.substring(indexMe + 3, data.indexOf(" ", indexMe + 3));
      velGrossaEsquerda = velEsquerdaStr.toInt();
      tratarValor(velGrossaEsquerda, velTratadaEsquerda);
      
      // Define a direção do motor esquerdo
      digitalWrite(DIR_ESQUERDA, velGrossaEsquerda >= 0 ? HIGH : LOW);
      Serial.print("Velocidade ajustada para o motor esquerdo: ");
      Serial.println(velTratadaEsquerda);
    }

    // Extrai o valor para o motor direito (Md)
    int indexMd = data.indexOf("Md ");
    if (indexMd != -1) {
      String velDireitaStr = data.substring(indexMd + 3);
      velGrossaDireita = velDireitaStr.toInt();
      tratarValor(velGrossaDireita, velTratadaDireita);
      
      // Define a direção do motor direito
      digitalWrite(DIR_DIREITA, velGrossaDireita >= 0 ? HIGH : LOW);
      Serial.print("Velocidade ajustada para o motor direito: ");
      Serial.println(velTratadaDireita);
    }
  }

  // Movimenta o motor esquerdo
  for (int i = 0; i < steps_per_rev; i++) {
    digitalWrite(STEP_ESQUERDA, HIGH);
    delayMicroseconds(velTratadaEsquerda);
    digitalWrite(STEP_ESQUERDA, LOW);
    delayMicroseconds(velTratadaEsquerda);
  }

  // Movimenta o motor direito
  for (int i = 0; i < steps_per_rev; i++) {
    digitalWrite(STEP_DIREITA, HIGH);
    delayMicroseconds(velTratadaDireita);
    digitalWrite(STEP_DIREITA, LOW);
    delayMicroseconds(velTratadaDireita);
  }

  delay(1000); // Pausa entre rotações
}