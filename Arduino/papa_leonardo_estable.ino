#define USE_USBCON
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

/* VNH2SP30 pin definitions */

double distanceleft;
double durationleft;
double distanceright;
double durationright;

// --- NUEVA ASIGNACIÓN DE PINES PARA SENSORES ULTRASÓNICOS ---
const int triggerPinright = 3;
const int echoPinright = 5;
const int triggerPinleft = 13;
const int echoPinleft = 11;

float c = 0.034;

// --- PINES DEL MOTOR SHIELD ---
int inApin[2] = {2, 7};  // INA: Clockwise input
int inBpin[2] = {4, 8};  // INB: Counter-clockwise input
int pwmpin[2] = {9, 10}; // PWM input
//int cspin[2] = {2, 3};   // CS: Current sense ANALOG input
//int enpin[2] = {0, 1};   // EN: Status of switches output (Analog pin)

// --- VARIABLES DE CONTROL ---
String msg = "";
bool modoAutomatico = false; // Comienza en modo manual por defecto (cambialo a true si prefieres)

void init_motor(void) {
  // Inicializa pines digitales como salidas
  for (int i=0; i<2; i++) {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Inicializa los motores frenados
  for (int i=0; i<2; i++) {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1) {
    if (direct <=4) {
      // Configurar inA
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Configurar inB
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
  
void test_adelante(void) {
    motorGo(LEFT_MOTOR, CW, 70); // LEFT es 1, RIGHT es 0 (según el original, motorGo(0) era 108)
    motorGo(RIGHT_MOTOR, CW, 75);
}

void test_atras(void) {
    motorGo(LEFT_MOTOR, CCW, 70);
    motorGo(RIGHT_MOTOR, CCW, 75);
}

void test_girar_derecha(void) {
    motorGo(LEFT_MOTOR, CW, 70);
    motorGo(RIGHT_MOTOR, CCW, 75);  
}

void test_girar_izquierda(void) {
    motorGo(LEFT_MOTOR, CCW, 70);
    motorGo(RIGHT_MOTOR, CW, 75);  
}

// Nueva función necesaria para el control manual
void test_frenar(void) {
    motorGo(LEFT_MOTOR, BRAKEGND, 0);
    motorGo(RIGHT_MOTOR, BRAKEGND, 0);
}

void setup() {
  init_motor();
  
  pinMode(echoPinleft, INPUT);
  pinMode(triggerPinleft, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(echoPinright, INPUT);
  pinMode(triggerPinright, OUTPUT);
  digitalWrite(A5, HIGH);
  
  Serial.begin(115200);
  Serial.println("Sistema Iniciado. Modo MANUAL activo.");
}

void loop() {  
  // 1. LECTURA DE COMANDOS SERIALES
  if (Serial.available()) {
    msg = Serial.readStringUntil('\n');
    msg.trim();

    // Cambiar de modo al recibir "L1_release"
    if (msg == "on_L1_release") {
      modoAutomatico = !modoAutomatico; // Invierte el estado (Manual <-> Automático)
      test_frenar(); // Frena los motores por seguridad al cambiar de modo
      
      Serial.print("Cambio de modo: ");
      Serial.println(modoAutomatico ? "AUTOMATICO" : "MANUAL");
    }
    
    // 2. LÓGICA DE CONDUCCIÓN MANUAL (Solo si el modo automático está apagado)
    else if (!modoAutomatico) {
      if (msg == "DIR,UP,1") {
        test_adelante();
      }
      else if (msg == "DIR,UP,0") {
        test_frenar();
      }
      else if (msg == "DIR,DOWN,1") {
        test_atras();
      }
      else if (msg == "DIR,DOWN,0") {
        test_frenar();
      }
      else if (msg == "DIR,LEFT,1") {
        test_girar_izquierda();
      }
      else if (msg == "DIR,LEFT,0") {
        test_frenar();
      }
      else if (msg == "DIR,RIGHT,1") {
        test_girar_derecha();
      }
      else if (msg == "DIR,RIGHT,0") {
        test_frenar();
      }
    }
  }

  // 3. LÓGICA DE CONDUCCIÓN AUTOMÁTICA (Solo si el modo automático está encendido)
  if (modoAutomatico) {
    // --- LECTURA SENSOR IZQUIERDO ---
    digitalWrite(triggerPinleft, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPinleft, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPinleft, LOW);

    durationleft = pulseIn(echoPinleft, HIGH);
    distanceleft = (durationleft * c) / 2;
    
    // --- LECTURA SENSOR DERECHO ---
    digitalWrite(triggerPinright, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPinright, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPinright, LOW);
    
    durationright = pulseIn(echoPinright, HIGH);
    distanceright = (durationright * c) / 2;
    
    // --- TOMA DE DECISIONES AUTOMÁTICA ---
    if(distanceleft > 30 && distanceright > 30) {
      test_adelante(); 
      // Serial.println("Ir para adelante"); // Comentado para no saturar el Serial
    }
    else if (distanceleft > 30 && distanceright <= 30) {
      test_girar_izquierda();
      // Serial.println("Girar para la izquierda");
    }
    else if (distanceleft <= 30 && distanceright > 30) {
      test_girar_derecha();
      // Serial.println("Girar para la derecha");
    }
    else if (distanceleft <= 30 && distanceright <= 30) {
      test_girar_derecha(); // Gira hasta liberar obstáculo
      // Serial.println("Girar hasta poder moverse");
    }

    delay(100); // Pequeña pausa requerida por la lectura de los sensores
  }
}
