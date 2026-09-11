#define USE_USBCON

#define BRAKEVCC 0
#define CW       1
#define CCW      2
#define BRAKEGND 3

#define CS_THRESHOLD 100

#define RIGHT_MOTOR 1
#define LEFT_MOTOR  0

#define VELOCIDAD 75
#define FACTOR_CORRECCION 1.2


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

bool modoAutomatico = false;


// ============================================================
// INICIALIZACION DE MOTORES
// ============================================================

void init_motor(void)
{
  for (int i = 0; i < 2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  // Motores frenados al iniciar
  for (int i = 0; i < 2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
    analogWrite(pwmpin[i], 0);
  }
}


// ============================================================
// CONTROL DE UN MOTOR
// ============================================================

void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <= 4)
    {
      // Configurar inA
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Configurar inB
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}


// ============================================================
// MOVIMIENTO MANUAL
// ============================================================

void test_adelante(void)
{
  motorGo(LEFT_MOTOR, CW, 75);
  motorGo(RIGHT_MOTOR, CW, 82);
}


void test_atras(void)
{
  motorGo(LEFT_MOTOR, CW, VELOCIDAD);
  motorGo(
    RIGHT_MOTOR,
    CW,
    (uint8_t)(VELOCIDAD * FACTOR_CORRECCION)
  );
}


void test_girar_derecha(void)
{
  motorGo(LEFT_MOTOR, CCW, VELOCIDAD);

  motorGo(
    RIGHT_MOTOR,
    CW,
    (uint8_t)(VELOCIDAD * FACTOR_CORRECCION)
  );
}


void test_girar_izquierda(void)
{
  motorGo(LEFT_MOTOR, CW, VELOCIDAD);

  motorGo(
    RIGHT_MOTOR,
    CCW,
    (uint8_t)(VELOCIDAD * FACTOR_CORRECCION)
  );
}


void test_frenar(void)
{
  motorGo(LEFT_MOTOR, BRAKEGND, 0);
  motorGo(RIGHT_MOTOR, BRAKEGND, 0);
}


// ============================================================
// CONTROL DE VELOCIDAD CON SIGNO
//
// pwm > 0  -> CW
// pwm < 0  -> CCW
// pwm = 0  -> freno
// ============================================================

void moverMotorSigned(uint8_t motor, int pwm)
{
  pwm = constrain(pwm, -255, 255);

  if (pwm > 0)
  {
    motorGo(
      motor,
      CW,
      (uint8_t)pwm
    );
  }

  else if (pwm < 0)
  {
    motorGo(
      motor,
      CCW,
      (uint8_t)abs(pwm)
    );
  }

  else
  {
    motorGo(
      motor,
      BRAKEGND,
      0
    );
  }
}


// ============================================================
// PROCESAR COMANDO NAV2
//
// Formato:
// VEL,left,right
//
// Ejemplos:
// VEL,80,80
// VEL,-60,60
// VEL,0,0
// ============================================================

void procesarVelocidad(String comando)
{
  int coma1 = comando.indexOf(',');

  int coma2 = comando.indexOf(
    ',',
    coma1 + 1
  );


  if (coma1 == -1 || coma2 == -1)
  {
    Serial.println("ERROR: formato VEL invalido");
    return;
  }


  int pwmLeft =
    comando.substring(
      coma1 + 1,
      coma2
    ).toInt();


  int pwmRight =
    comando.substring(
      coma2 + 1
    ).toInt();


  pwmLeft = constrain(
    pwmLeft,
    -255,
    255
  );

  pwmRight = constrain(
    pwmRight,
    -255,
    255
  );


  moverMotorSigned(
    LEFT_MOTOR,
    pwmLeft
  );

  moverMotorSigned(
    RIGHT_MOTOR,
    pwmRight
  );


  // Para debug
  Serial.print("VEL -> L: ");
  Serial.print(pwmLeft);

  Serial.print(" R: ");
  Serial.println(pwmRight);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  init_motor();

  Serial.begin(115200);

  test_frenar();

  Serial.println(
    "Sistema iniciado. Modo MANUAL activo."
  );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  if (Serial.available())
  {
    msg = Serial.readStringUntil('\n');

    msg.trim();


    // ========================================================
    // CAMBIO MANUAL <-> AUTOMATICO
    // ========================================================

    if (msg == "on_L1_release")
    {
      modoAutomatico = !modoAutomatico;

      // Siempre frenar al cambiar de modo
      test_frenar();


      Serial.print("Cambio de modo: ");

      if (modoAutomatico)
      {
        Serial.println("AUTOMATICO NAV2");
      }
      else
      {
        Serial.println("MANUAL");
      }
    }


    // ========================================================
    // MODO MANUAL
    // ========================================================

    else if (!modoAutomatico)
    {
      if (msg == "DIR,UP,1")
      {
        test_adelante();
      }

      else if (msg == "DIR,UP,0")
      {
        test_frenar();
      }

      else if (msg == "DIR,DOWN,1")
      {
        test_atras();
      }

      else if (msg == "DIR,DOWN,0")
      {
        test_frenar();
      }

      else if (msg == "DIR,LEFT,1")
      {
        test_girar_izquierda();
      }

      else if (msg == "DIR,LEFT,0")
      {
        test_frenar();
      }

      else if (msg == "DIR,RIGHT,1")
      {
        test_girar_derecha();
      }

      else if (msg == "DIR,RIGHT,0")
      {
        test_frenar();
      }
    }


    // ========================================================
    // MODO AUTOMATICO NAV2
    // ========================================================

    else
    {
      if (msg.startsWith("VEL,"))
      {
        procesarVelocidad(msg);
      }
    }
  }
}
