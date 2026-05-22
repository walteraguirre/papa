#define USE_USBCON
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#include <Servo.h>

int distance1;
double duration1;

int distance2;
double duration2;

int valor;

// Pines ultrasonido modificados para no pisar motores
const int echoPin1 = 11;
const int echoPin2 = 12;
const int triggerPin1 = 5;
const int triggerPin2 = A0;

const int ledPin = 13;

float c = 0.034;

int inApin[2] = {2, 7};    // INA
int inBpin[2] = {4, 8};    // INB
int pwmpin[2] = {9, 10};   // PWM

// No usados por ahora
int cspin[2] = {2, 3};
int enpin[2] = {0, 1};

int statpin = 13;

void init_motor(void) {
  for (int i = 0; i < 2; i++) {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  for (int i = 0; i < 2; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1) {
    if (direct <= 4) {
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}

void test_adelante(void) {
  motorGo(0, CW, 100);
  motorGo(1, CW, 100);
}

void test_atras(void) {
  motorGo(0, CCW, 100);
  motorGo(1, CCW, 100);
}

void test_girar_derecha(void) {
  motorGo(LEFT_MOTOR, CW, 100);
  motorGo(RIGHT_MOTOR, CCW, 100);
}

void test_girar_izquierda(void) {
  motorGo(LEFT_MOTOR, CCW, 100);
  motorGo(RIGHT_MOTOR, CW, 100);
}

void setup() {
  init_motor();

  pinMode(echoPin1, INPUT);
  pinMode(triggerPin1, OUTPUT);

  pinMode(echoPin2, INPUT);
  pinMode(triggerPin2, OUTPUT);

  Serial.begin(115200);
}

void loop() {
  // Ultrasonido 1
  digitalWrite(triggerPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin1, LOW);

  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * c / 2;

  delayMicroseconds(50);

  // Ultrasonido 2
  digitalWrite(triggerPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin2, LOW);

  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * c / 2;

  Serial.print("Distance1: ");
  Serial.print(distance1);
  Serial.println(" cm");

  Serial.print("Distance2: ");
  Serial.print(distance2);
  Serial.println(" cm");

  if (distance1 > 15 && distance2 > 15) {
    Serial.println("Adelante");
    //test_adelante();
  }
  else if (distance1 > 15 && distance2 <= 15) {
    Serial.println("Izquierda");
    //test_girar_izquierda();
  }
  else if (distance1 <= 15 && distance2 > 15) {
    Serial.println("Derecha");
    //test_girar_derecha();
  }
  else if (distance1 <= 15 && distance2 <= 15) {
    Serial.println("Atras");
    //test_atras();
  }

  delay(1000);
}