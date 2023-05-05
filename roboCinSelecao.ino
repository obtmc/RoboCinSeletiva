#include <math.h>

//chanel A e B servem para leitura do encoder
#define CHANNEL_A 2
#define CHANNEL_B 3

//pinos para controle do motor
#define MOTOR_INPUT1 10
#define MOTOR_INPUT2 11
#define MOTOR_SPEED 9

// Varia de -16.31 a 16.31, valor encontrado no teste prático
float motorRPMDesejado = 10;
float motorRPMAlternative = 5;
float lastErrorRPM = 0;

const float kMotor = 255/16.31;

// varia de 0 a 255 e quanto maior mais rápido ele gira
float motorPWMAplicado = motorRPMDesejado*kMotor;
float motorPWMAlternative = motorRPMAlternative*kMotor;
float motorPWM = motorPWMAplicado;

float aux = 0;


// Número de pulsos do encoder por rotação do motor
const int ENCODER_PULSES_PER_ROTATION = 4096;

const float raioDaRoda = 0.00026; //Em kilometros
const float comprimentoDaRoda = 2 * M_PI * raioDaRoda;

// Número de pulsos do encoder por rotação do motor
// Uso de votatile para Informar ao compilador que o valor da variável pode mudar a qualquer momento 
// O compilador deverá sempre buscar a variável na memória em vez de manter uma cópia em um registrador
volatile long encoderCount = 0; 
volatile long lastEncoderCount = 0;
long currentEncoderCount = 0;
long encoderDelta = 0;
int count = 0;

long lastExecutionTime = 0;
long executionInterval = 100;


bool isMotorOn = true;  // variável para controlar o motor

void handleEncoderInterrupt()
{
  // Lê o estado atual dos pinos do encoder
  int channelAState = digitalRead(CHANNEL_A);
  int channelBState = digitalRead(CHANNEL_B);

  // Detecta as variações de sinal nos pinos do encoder
  if (channelAState != channelBState) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}

void setup()
{
  // Configurando portas para controle do motor
  pinMode(CHANNEL_A, INPUT_PULLUP);
  pinMode(CHANNEL_B, INPUT_PULLUP);
  pinMode(MOTOR_INPUT1, OUTPUT);
  pinMode(MOTOR_INPUT2, OUTPUT);
  pinMode(MOTOR_SPEED, OUTPUT);
  
  // Configura a interrupção para o pino do encoder
  attachInterrupt(digitalPinToInterrupt(CHANNEL_A), handleEncoderInterrupt, CHANGE);
  
  // Controla a velocidade atual do motor
  analogWrite(MOTOR_SPEED, motorPWM);
  
  Serial.begin(9600); 
}

void loop()
{
  // Controla o sentido do motor
  digitalWrite(MOTOR_INPUT1, HIGH);
  digitalWrite(MOTOR_INPUT2, LOW);
  
  
  
  if(count > 39){
    float motorPWMAux = motorPWMAplicado;
    motorPWMAplicado = motorPWMAlternative;
    motorPWMAlternative = motorPWMAux;
    
    motorPWM = motorPWMAplicado; //motorPWM será alterado ao longo do ciclo
    
    float motorRPMAux = motorRPMDesejado;
    motorRPMDesejado = motorRPMAlternative;
    motorRPMAlternative = motorRPMAux;
    
    count = 0;
  }
  
  //unsigned long now = millis();
  //long executionInterval = now - lastExecutionTime;
  //lastExecutionTime = now;
  
  // Calcula a velocidade atual do motor
  currentEncoderCount = encoderCount;
  encoderDelta = currentEncoderCount - lastEncoderCount;
    
  // Atualiza lastEncoderCount
  lastEncoderCount = currentEncoderCount;
 
  // Multiplica por 60 para ter a quantidade de rotações por minuto RPM e divide por 0,1 JÁ QUE A INFORMAÇÃO DO ENCODER ESTÁ SENDO VERIFICADA A CADA 0,1s = 100 milesegundos 
  float motorRPM = ((float)encoderDelta / ENCODER_PULSES_PER_ROTATION) * 60.0 /0.1;//((float)executionInterval/1000);
  
  // Calucula cooreção de velocidade
  float errorRPM = motorRPMDesejado - motorRPM;
  
  float errorVariacao = lastErrorRPM - errorRPM;
  motorPWM = motorPWM + errorRPM*kMotor*(errorVariacao/10000); // Penso que a correção precisa ser corrigida por um valor maior quanto menor for a velocidade desejada e menor quanto for a velocidade desejada
  lastErrorRPM = errorRPM;
  
  // Controla a velocidade atual do motor
  analogWrite(MOTOR_SPEED, motorPWM);
  
  
  //Calcula velocidade do veiculo com base na velociadade de rotação e no comprimento da roda do veículo
  //float speedVehicle = motorRPM * comprimentoDaRoda * 60;
  
  //float newSpeedVehicle = 1;
  //float intencaoMotorRPM = newSpeedVehicle/(comprimentoDaRoda*60);
  
  // Imprime a velocidade de rotação atual do motor no monitor serial
  //Serial.print("Motor RPM: ");
  Serial.println(motorRPM);
  count++;
  delay(99); //100 - tempo de execução sem delay
}