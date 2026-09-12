#define USE_USBCON

#define BRAKEVCC 0
#define CW       1
#define CCW      2
#define BRAKEGND 3

#define RIGHT_MOTOR 1
#define LEFT_MOTOR  0

// Factor de corrección para el motor derecho
#define FACTOR_CORRECCION 1.2

// Tiempo máximo sin recibir comandos antes de detenerse (milisegundos)
#define TIMEOUT_SEGURIDAD 1000

// ============================================================
// PINES DEL MOTOR SHIELD
// ============================================================
int inApin[2] = {2, 7};
int inBpin[2] = {4, 8};
int pwmpin[2] = {9, 10};

// ============================================================
// VARIABLES DE CONTROL
// ============================================================
String msg = "";
unsigned long ultimoComandoRecibido = 0;
bool motoresFrenados = true;

// ============================================================
// INICIALIZACION DE MOTORES
// ============================================================
void init_motor(void) {
  for (int i = 0; i < 2; i++) {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  frenarMotores();
}

// ============================================================
// CONTROL DE MOTOR BASE
// ============================================================
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm) {
  if (motor <= 1 && direct <= 4) {
    if (direct <= 1) digitalWrite(inApin[motor], HIGH);
    else digitalWrite(inApin[motor], LOW);

    if ((direct == 0) || (direct == 2)) digitalWrite(inBpin[motor], HIGH);
    else digitalWrite(inBpin[motor], LOW);

    analogWrite(pwmpin[motor], pwm);
  }
}

// ============================================================
// DETENCIÓN TOTAL
// ============================================================
void frenarMotores(void) {
  motorGo(LEFT_MOTOR, BRAKEGND, 0);
  motorGo(RIGHT_MOTOR, BRAKEGND, 0);
  motoresFrenados = true;
}

// ============================================================
// CONTROL DE VELOCIDAD CON SIGNO Y CORRECCIÓN
// ============================================================
void moverMotorSigned(uint8_t motor, int pwm) {
  // Aplicar factor de corrección si es el motor derecho
  if (motor == RIGHT_MOTOR) {
    pwm = (int)(pwm * FACTOR_CORRECCION);
  }

  // Limitar el PWM al máximo eléctrico
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0) {
    motorGo(motor, CW, (uint8_t)pwm);
  } 
  else if (pwm < 0) {
    motorGo(motor, CCW, (uint8_t)abs(pwm));
  } 
  else {
    motorGo(motor, BRAKEGND, 0);
  }
}

// ============================================================
// PROCESAR COMANDO SERIAL (VEL,left,right)
// ============================================================
void procesarVelocidad(String comando) {
  int coma1 = comando.indexOf(',');
  int coma2 = comando.indexOf(',', coma1 + 1);

  if (coma1 == -1 || coma2 == -1) return;

  int pwmLeft = comando.substring(coma1 + 1, coma2).toInt();
  int pwmRight = comando.substring(coma2 + 1).toInt();

  moverMotorSigned(LEFT_MOTOR, pwmLeft);
  moverMotorSigned(RIGHT_MOTOR, pwmRight);

  // Actualizar el watchdog
  ultimoComandoRecibido = millis();
  motoresFrenados = false;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  init_motor();
  Serial.begin(115200);
}

// ============================================================
// LOOP PRINCIPAL
// ============================================================
void loop() {
  // 1. Recepción y procesamiento de comandos
  if (Serial.available()) {
    msg = Serial.readStringUntil('\n');
    msg.trim();
    
    if (msg.startsWith("VEL,")) {
      procesarVelocidad(msg);
    }
  }

  // 2. Watchdog de seguridad (Timeout)
  if (!motoresFrenados && (millis() - ultimoComandoRecibido > TIMEOUT_SEGURIDAD)) {
    frenarMotores();
  }
}
