
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>  //pca9685
#include <Servo.h> 

#include <EnableInterrupt.h>

#define pin_INT_Throttle 11  // Pin Throttle
#define pin_INT_Yaw 10      // Pin Yaw       // Varilla 7 timón
#define pin_INT_Pitch 12   // Pin Pitch     // Varilla 11 profundidad
#define pin_INT_Roll 9    // Pin Roll      // Varilla 15 balanceo
#define status_led 13    // Off: starting, Blink: calbration mode, On: Flight mode

#define pin_esc 5


Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);
long loop_timer, tiempo_ejecucion;

int Vel = 100;
int Time = 0;
float P = 0.75;

int tvel = 1000;
boolean tcontrol=false; 
Servo ESC;

unsigned int pos0=172;    // ancho de pulso en cuentas para pocicion 0°
unsigned int pos180=565; // ancho de pulso en cuentas para la pocicion 180°

// Maps


// fin maps

void setServo(uint8_t n_servo, int angulo) {
  int duty;
  duty=map(angulo, 0, 180, pos0, pos180);
  servos.setPWM(n_servo, 0, duty);  
}

int mapYawToServoPosition(int yawValue) {
  int servoPosition;

  servoPosition = map(yawValue, 970, 1980, 65, 115); // Mapear de 50 grados a 130 grados (180 - 50)
  servoPosition = constrain(servoPosition, 65, 115); // Limitar el rango a 50 grados

  return servoPosition;
}

/* old : todo el angulo para lso servos
int mapYawToServoPosition(int yawValue) {
  int servoPosition;
  servoPosition = map(yawValue, 970, 1980, 0, 180);
  servoPosition = constrain(servoPosition, 0, 180);
  return servoPosition;
}
*/

int mapPitchToServoPosition(int pitchValue) {
  int servoPosition;
  servoPosition = map(pitchValue, 1104, 1930, 65, 115); // Mapear de 50 grados a 130 grados (180 - 50)
  servoPosition = constrain(servoPosition, 65, 115); // Limitar el rango a 50 grados
  return servoPosition;
}

int mapRollToServoPosition(int rollValue) {
  int servoPosition;
  servoPosition = map(rollValue, 1040, 1960, 65, 115); // Mapear de 50 grados a 130 grados (180 - 50)
  servoPosition = constrain(servoPosition, 65, 115); // Limitar el rango a 50 grados
  return servoPosition;
}

void setThrottle(int throttleValue) {
  int mappedThrottle = map(throttleValue, 1060, 1712, 950, 2000);
  servos.setPWM(3, 0, mappedThrottle);
}

// Interrupciones


// INTERRUPCIÓN MANDO RC --> THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH) Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW)  RC_Throttle_raw  = micros() - Throttle_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH) Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW)  RC_Pitch_raw  = micros() - Pitch_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> ROLL
volatile long Roll_HIGH_us;
volatile int RC_Roll_raw;
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH) Roll_HIGH_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW)  RC_Roll_raw  = micros() - Roll_HIGH_us;
}

// INTERRUPCIÓN MANDO RC --> YAW
volatile long Yaw_HIGH_us;
volatile int RC_Yaw_raw;
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH) Yaw_HIGH_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW)  RC_Yaw_raw  = micros() - Yaw_HIGH_us;
}

// fin interrupciones


void setup() {
  servos.begin();  
  servos.setPWMFreq(60); //Frecuecia PWM de 60Hz o T=16,66ms

  // Declaración de interrupciones
  pinMode(pin_INT_Yaw, INPUT_PULLUP);                   // YAW
  enableInterrupt(pin_INT_Yaw, INT_Yaw, CHANGE);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);              // Throttle
  enableInterrupt(pin_INT_Throttle, INT_Throttle, CHANGE);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);                 // PITCH
  enableInterrupt(pin_INT_Pitch, INT_Pitch, CHANGE);
  pinMode(pin_INT_Roll, INPUT_PULLUP);                  // ROLL
  enableInterrupt(pin_INT_Roll, INT_Roll, CHANGE);

  pinMode(status_led, OUTPUT);
  //pinMode(pin_esc, OUTPUT);
  //analogWrite(pin_esc, 100);

  //ESC.attach(pin_esc);
  //ESC.writeMicroseconds(1000);
  //delay(3000);
  //ESC.writeMicroseconds(2000);

  Serial.begin(115200);
}

void loop() {

  while (micros() - loop_timer < 10000);
  tiempo_ejecucion = (micros() - loop_timer) / 1000;
  loop_timer = micros();

  // Monitor Serie
  Serial.print(RC_Throttle_raw);
  Serial.print("\t");
  Serial.print(RC_Pitch_raw);
  Serial.print("\t");
  Serial.print(RC_Roll_raw);
  Serial.print("\t");
  Serial.println(RC_Yaw_raw);

  setServo(7, mapYawToServoPosition(RC_Yaw_raw));
  setServo(11, mapPitchToServoPosition(RC_Pitch_raw));
  setServo(15, mapRollToServoPosition(RC_Roll_raw));
  setThrottle(RC_Throttle_raw);

// 1060, 1712
  if (RC_Throttle_raw <= 1350)
  {
    digitalWrite(status_led, HIGH);
  }
  else
  {
    digitalWrite(status_led, LOW);
  }
}
