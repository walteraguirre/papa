#define USE_USBCON
#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100
#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1
/*  VNH2SP30 pin definitions

 xxx[0] controls '1' outputs
 xxx[1] controls '2' outputs */
//#include <Servo.h>

double distanceleft;
double durationleft;

double distanceright;
double durationright;

int valor;

const int echoPinleft = 11;
const int echoPinright = 13;
const int triggerPinleft = 5;
const int triggerPinright = A0;

//const int ledPin = 13;


float c = 0.034;
int inApin[2] = {2, 7};  // INA: Clockwise input
int inBpin[2] = {4, 8}; // INB: Counter-clockwise input
int pwmpin[2] = {9, 10}; // PWM input

int cspin[2] = {2, 3}; // CS: Current sense ANALOG input
int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)

//int statpin = 13;

void init_motor(void) {
  // Initialize digital pins as outputs
  for (int i=0; i<2; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }
  // Initialize braked
  for (int i=0; i<2; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
  }
}


/* motorGo() will set a motor going in a specific direction
 the motor will continue going in that direction, at that speed
 until told to do otherwise.
 
 motor: this should be either 0 or 1, will selet which of the two
 motors to be controlled
 
 direct: Should be between 0 and 3, with the following result
 0: Brake to VCC
 1: Clockwise
 2: CounterClockwise
 3: Brake to GND
 
 pwm: should be a value between ? and 1023, higher the number, the faster
 it'll go
 */
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
  if (motor <= 1)
  {
    if (direct <=4)
    {
      // Set inA[motor]
      if (direct <=1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct==0)||(direct==2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
  }
}
  
void test_adelante(void)
{
    motorGo(0, CW, 100);
    motorGo(1, CW, 100);
}

void test_atras(void)
{
    motorGo(0, CCW, 100);
    motorGo(1, CCW, 100);
}

void test_girar_derecha(void)
{
    motorGo(LEFT_MOTOR, CW, 100);
    motorGo(RIGHT_MOTOR, CCW, 100);  
}

void test_girar_izquierda(void)
{
    motorGo(LEFT_MOTOR, CCW, 100);
    motorGo(RIGHT_MOTOR, CW, 100);  
}
void setup()
{
  init_motor();
  pinMode(A5, OUTPUT);
  digitalWrite(A5, HIGH);
  pinMode(echoPinleft, INPUT);
  pinMode(triggerPinleft, OUTPUT);
  pinMode(echoPinright, INPUT);
  pinMode(triggerPinright, OUTPUT);
  //pinMode(ledPin, OUTPUT);
  Serial.begin(115200);

}

void loop() 
{  
  digitalWrite(triggerPinleft, LOW);
  delayMicroseconds(10);
  digitalWrite(triggerPinleft, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinleft, LOW);

  durationleft = pulseIn(echoPinleft, HIGH);
  distanceleft = durationleft*c/2;
  
  
  digitalWrite(triggerPinright, LOW);
  delayMicroseconds(10);
  digitalWrite(triggerPinright, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPinright, LOW);
  
  durationright = pulseIn(echoPinright, HIGH);
  distanceright = durationright*c/2;
  
  Serial.print("Distance left sensor: ");
  Serial.print(distanceleft);
  Serial.println("cm");
  
  Serial.print("Distance right sensor: ");
  Serial.print(distanceright);
  Serial.println("cm");
  


  if(distanceleft > 30 && distanceright > 30){
    test_adelante(); 
    Serial.println("ir para adelante");
  }
  else if (distanceleft > 30 && distanceright <= 30){
    test_girar_izquierda();
    Serial.println("girar para la izquierda");
  }
  else if (distanceleft <= 30 && distanceright > 30){
    test_girar_derecha();
    Serial.println("girar para la derecha");
  }
  else if (distanceleft <= 30 && distanceright <= 30){
    test_girar_derecha();
    Serial.println("Girar hasta poder moverse");
  }

  delay(100);
  
}
