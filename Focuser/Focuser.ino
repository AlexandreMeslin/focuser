/*
 * Pinouts
 *  0 RX 
 *  1 TX
 *  2  
 *  3 CLK
 *  4 DT
 *  5 SW
 *  6 
 *  7 
 *  8 
 *  9 Stepper 0
 * 10 Stepper 1
 * 11 Stepper 2
 * 12 Stepper 3
 * 13
 * A0 
 * A1 
 * A2
 * A3
 * A4
 * A5
 */

//#include <Stepper.h>

/**
 * Protótipos
 */
void step(int direcao, int nSteps);
void step(int direcao);

/**
 * Stepper
 */
const int stepsPerRevolution = 2048;  // the number of steps per revolution
//Stepper myStepper(stepsPerRevolution, 9, 10, 11, 12);
//#define STEP  2048
#define STEPPER0 9
#define STEPPER1 10
#define STEPPER2 11
#define STEPPER3 12
struct {
  int posicao;
  int nPosicoes;
  int mControl[4][4];
} stepper = {
  0,
  4,
  {
    {1,0,0,1},
    {0,0,1,1},
    {0,1,1,0},
    {1,1,0,0}
  }
};

/**
 * Rotary Encoder KY-040
 */
const int pinoCLK = 3;  // CLK
const int pinoDT = 4;   // DT
const int pinoSW = 5;   // SW

int contadorPos = 0;
int ultPosicao; 
int leituraCLK;
boolean bCW;

void setup() {
  Serial.begin (9600);

  // Rotary Encoder
  pinMode(pinoCLK,INPUT);
  pinMode(pinoDT,INPUT);
  pinMode(pinoSW,INPUT_PULLUP);
  ultPosicao = digitalRead(pinoCLK);

  // Stepper motor
  pinMode(STEPPER0, OUTPUT);
  pinMode(STEPPER1, OUTPUT);
  pinMode(STEPPER2, OUTPUT);
  pinMode(STEPPER3, OUTPUT);
} 

 void loop() {
//  myStepper.setSpeed(100);

  leituraCLK = digitalRead(pinoCLK);
  int leituraDT = digitalRead(pinoDT);
  if(leituraCLK != ultPosicao) {
    Serial.print(ultPosicao);
    Serial.print(" ");
    Serial.print(leituraCLK);
    Serial.print(" ");
    Serial.println(leituraDT);
    if(ultPosicao == 1)
    if (leituraDT != leituraCLK) {
      // sentido horário
      Serial.println("Sentido horário");
      contadorPos++; 
      bCW = true;
//      myStepper.step(STEP);
      if(digitalRead(pinoSW) == LOW) {
        step(1, 10);
      } else {
        step(1);      
      }
    } else {
      // sentido anti-horário
      Serial.println("Sentido anti-horário");
      bCW = false;
      contadorPos--;
//      myStepper.step(-STEP);
      if(digitalRead(pinoSW) == LOW) {
        step(-1, 10);
      } else {
        step(-1);      
      }
    }
  }
  
  ultPosicao = leituraCLK;
  delay(4);
}

void step(int direcao, int nSteps) {
  for(int i=0; i<nSteps; i++) step(direcao);
}
void step(int direcao) {
  stepper.posicao = (stepper.posicao + direcao) & (stepper.nPosicoes-1);
  Serial.print("Posição: "); Serial.println(stepper.posicao);
  digitalWrite(STEPPER0, stepper.mControl[stepper.posicao][0]);
  digitalWrite(STEPPER1, stepper.mControl[stepper.posicao][1]);
  digitalWrite(STEPPER2, stepper.mControl[stepper.posicao][2]);
  digitalWrite(STEPPER3, stepper.mControl[stepper.posicao][3]);
  delay(4);
}
