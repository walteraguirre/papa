#include <Arduino.h>
#include <Wire.h>

// Pines Ultrasonidos
const int trigL = 33, echoL = 32;
const int trigC = 23, echoC = 22;
const int trigR = 21, echoR = 19;

// Pines y Dirección del MPU6050
const int SDA_PIN = 17;
const int SCL_PIN = 16;
const int MPU_ADDR = 0x68;

// Pines Encoders (Canal A y B)
const int ENC1_A = 26;
const int ENC1_B = 13;
const int ENC2_A = 27;
const int ENC2_B = 14;

// Contadores volátiles (modificados dentro de las interrupciones)
volatile long countEnc1 = 0;
volatile long countEnc2 = 0;

// Rutinas de interrupción alojadas en RAM (IRAM_ATTR) para máxima velocidad
void IRAM_ATTR isrEnc1() {
  if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) {
    countEnc1++;
  } else {
    countEnc1--;
  }
}

void IRAM_ATTR isrEnc2() {
  if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) {
    countEnc2++;
  } else {
    countEnc2--;
  }
}

void setup() {
  Serial.begin(115200);
  
  pinMode(trigL, OUTPUT); pinMode(echoL, INPUT);
  pinMode(trigC, OUTPUT); pinMode(echoC, INPUT);
  pinMode(trigR, OUTPUT); pinMode(echoR, INPUT);

  // Configuración de encoders con resistencias Pull-Up internas
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  // Disparar la interrupción cuando el Canal A pasa de BAJO a ALTO (RISING)
  attachInterrupt(digitalPinToInterrupt(ENC1_A), isrEnc1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), isrEnc2, RISING);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
}

long getDistance(int trig, int echo) {
  digitalWrite(trig, LOW); delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  
  long duration = pulseIn(echo, HIGH, 30000); 
  if (duration == 0) return 400; 
  return duration * 0.034 / 2;
}

void loop() {
  // --- 1. LECTURA DE ULTRASONIDOS ---
  long distL = getDistance(trigL, echoL); delay(10);
  long distC = getDistance(trigC, echoC); delay(10);
  long distR = getDistance(trigR, echoR); delay(10);

  // --- 2. LECTURA DEL GIROSCOPIO ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true); 

  int16_t gX = (Wire.read() << 8) | Wire.read();
  int16_t gY = (Wire.read() << 8) | Wire.read();
  int16_t gZ = (Wire.read() << 8) | Wire.read();

  float gyroX = gX / 131.0;
  float gyroY = gY / 131.0;
  float gyroZ = gZ / 131.0;

  // --- 3. LECTURA SEGURA DE ENCODERS ---
  // Se desactivan momentáneamente las interrupciones para evitar que 
  // el valor cambie justo en medio de la copia a las variables locales.
  noInterrupts(); 
  long currentEnc1 = countEnc1;
  long currentEnc2 = countEnc2;
  interrupts();   

  // --- 4. ENVÍO DE DATOS CSV ---
  // Trama actual: distL,distC,distR,gyroX,gyroY,gyroZ,enc1,enc2
  Serial.print(distL); Serial.print(",");
  Serial.print(distC); Serial.print(",");
  Serial.print(distR); Serial.print(",");
  Serial.print(gyroX, 2); Serial.print(","); 
  Serial.print(gyroY, 2); Serial.print(",");
  Serial.print(gyroZ, 2); Serial.print(",");
  Serial.print(currentEnc1); Serial.print(",");
  Serial.println(currentEnc2);

  delay(30);
}