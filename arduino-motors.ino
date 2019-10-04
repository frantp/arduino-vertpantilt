#include <Wire.h>
#include <Servo.h>
#include "DualMC33926MotorShield.h"


// ****************************************************************************
// Constants
// ****************************************************************************

// Pins
const byte ENCODER_A_PIN     =    2;
const byte ENCODER_B_PIN     =    3;
const byte ENCODER_INT_PIN   =    2;
const byte MOTOR_PAN_PIN     =   11;
const byte MOTOR_TILT_PIN    =   10;
const byte FEEDBACK_PAN_PIN  =   A2;
const byte FEEDBACK_TILT_PIN =   A1;

// I2C address
const byte I2C_ADDR          = 0x16;

// Commands
const byte CMD_MOVE_VERT     = 0xA0;
const byte CMD_MOVE_PAN      = 0xA1;
const byte CMD_MOVE_TILT     = 0xA2;
const byte CMD_READ_STATE    = 0xB0;

// Motors
const word MOTOR_MAX_MM      = 1400;
const word MOTOR_MM2ENCODER  =  250;
const word MOTOR_SPEED       =  200;
const word SERVO_DELAY       =   10;

// Structures
struct State {
  volatile unsigned long vert;
  byte pan, tilt, flags, bat1Voltage, bat2Voltage;
};


// ****************************************************************************
// Variables
// ****************************************************************************

DualMC33926MotorShield motorVert;
Servo motorPan, motorTilt;
byte cmd = CMD_READ_STATE;
byte pos = 0;
State state = { 0, 0, 0, 0, 0, 0 };


// ****************************************************************************
// Functions
// ****************************************************************************

// Setup method
void setup() {
  // Pins
  pinMode(ENCODER_A_PIN, INPUT);
  digitalWrite(ENCODER_A_PIN, HIGH);
  pinMode(ENCODER_B_PIN, INPUT);
  digitalWrite(ENCODER_B_PIN, HIGH);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_INT_PIN),
    readEncoder, FALLING);

  // Motors
  motorVert.init();
  motorPan.attach(MOTOR_PAN_PIN);  
  motorTilt.attach(MOTOR_TILT_PIN);

  // Wire and serial
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  // Move motors to position that is read from feedback
  const byte panPos  = analogRead(FEEDBACK_PAN_PIN ) * 10 / 42;
  const byte tiltPos = analogRead(FEEDBACK_TILT_PIN) * 10 / 42;
  motorPan.write(panPos);
  motorTilt.write(tiltPos);
  Serial.print("pan: ");
  Serial.print(panPos);
  Serial.print(", tilt: ");
  Serial.print(tiltPos);
  Serial.println();
}

// Loop method
void loop() {
  if (cmd != CMD_READ_STATE) {
    Serial.print("ACT: ");
    Serial.print(cmdChar(cmd));
    Serial.print(pos);
    Serial.println();
  }
  switch (cmd) {
    case CMD_MOVE_VERT: {
      Serial.print("cm");
      word mmPos = pos * 10;
      if (mmPos > MOTOR_MAX_MM) mmPos = MOTOR_MAX_MM;
      moveMotor(motorVert, state.vert, mmPos * MOTOR_MM2ENCODER);
    } break;
    case CMD_MOVE_PAN: {
      Serial.print("º");
      moveServo(motorPan, state.pan, pos);
    } break;
    case CMD_MOVE_TILT: {
      Serial.print("º");
      moveServo(motorTilt, state.tilt, pos);
    } break;
  }
  if (cmd != CMD_READ_STATE) {
    Serial.println("DONE");
  }
}

// Moves a motor
void moveMotor(DualMC33926MotorShield& motor,
               volatile unsigned long& current, int pos) {
  if (current < pos) {
    motor.setM1Speed(-MOTOR_SPEED);
    while (current < pos);
  } else if (current > pos) {
    motor.setM1Speed(+MOTOR_SPEED);
    while (current > pos);
  }
  motor.setM1Speed(0);
}


// Moves a servo
void moveServo(Servo& motor, byte& current, byte pos) {
  current = motor.read();
  const int update = current < pos ? +1 : -1;
  for (; current != pos; current += update) {
      delay(SERVO_DELAY);
      motor.write(current);
  }
}

// Reads the encoder values
void readEncoder() {
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    // Down
    if (state.vert > 0)
      state.vert -= 1;
  } else {
    // Up
    if (state.vert < MOTOR_MAX_MM * MOTOR_MM2ENCODER)
      state.vert += 1;
  }
}

// Receives data
void receiveEvent(int howMany) {
  // Read input
  if (!Wire.available()) {
    Serial.println("ERROR: No data to read");
    return;
  }
  cmd = Wire.read();
  if (cmd == CMD_READ_STATE) {
    return;
  }
  if (!Wire.available()) {
    Serial.println("ERROR: No position specified");
    return;
  }
  pos = Wire.read();

  // Print information
  Serial.print(cmdChar(cmd));
  if (cmd != CMD_READ_STATE) {
    Serial.print(" ");
    Serial.print(pos);
  }
  Serial.println();
}

// Receives a request for data
void requestEvent() {
  // Check read state
  if (cmd != CMD_READ_STATE) {
    Serial.println("ERROR: Read not requested");
    return;
  }
  // Serialize
  byte msg[] = {
    (byte)(state.vert / MOTOR_MM2ENCODER / 10),
    state.pan, state.tilt, state.flags,
    state.bat1Voltage, state.bat2Voltage
  };
  // Send
  Wire.write(msg, 6);
}

// Converts a command to a printable character
char cmdChar(byte cmd) {
  switch (cmd) {
    case CMD_READ_STATE: return 'R';
    case CMD_MOVE_VERT:  return 'V';
    case CMD_MOVE_PAN:   return 'P';
    case CMD_MOVE_TILT:  return 'T';
  }
  Serial.println("ERROR: Unknown command");
  return '-';
}
