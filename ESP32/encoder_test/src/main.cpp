#include <Arduino.h>

// ============================================================
// PINES ENCODERS
// ============================================================

// Encoder 1
const int ENC1_A = 26;
const int ENC1_B = 13;

// Encoder 2
const int ENC2_A = 27;
const int ENC2_B = 14;

// ============================================================
// CONTADORES
// ============================================================

volatile long countEnc1 = 0;
volatile long countEnc2 = 0;

// ============================================================
// INTERRUPCIONES
// ============================================================

void IRAM_ATTR isrEnc1()
{
    if (digitalRead(ENC1_A) == digitalRead(ENC1_B))
        countEnc1++;
    else
        countEnc1--;
}

void IRAM_ATTR isrEnc2()
{
    if (digitalRead(ENC2_A) == digitalRead(ENC2_B))
        countEnc2++;
    else
        countEnc2--;
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);

    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENC1_A), isrEnc1, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), isrEnc2, RISING);

    Serial.println("=== PRUEBA DE ENCODERS ===");
    Serial.println("Gira una rueda exactamente 1 vuelta.");
    Serial.println("Enc1\tEnc2");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    noInterrupts();

    long enc1 = countEnc1;
    long enc2 = countEnc2;

    interrupts();

    Serial.print(enc1);
    Serial.print("\t");
    Serial.println(enc2);

    delay(200);
}