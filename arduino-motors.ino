#include <Wire.h>
#include <Servo.h>
#include "DualMC33926MotorShield.h"


// ****************************************************************************
// Constants
// ****************************************************************************

// Pins
const byte ENCODER_A_PIN          =    2;
const byte ENCODER_B_PIN          =    3;
const byte ENCODER_INT_PIN        =    2;
const byte MOTOR_PAN_PIN          =   11;
const byte MOTOR_TILT_PIN         =   10;
const byte FEEDBACK_PAN_PIN       =   A2;
const byte FEEDBACK_TILT_PIN      =   A1;
const byte LOWER_LIMIT_SWITCH_PIN =   A6;
const byte UPPER_LIMIT_SWITCH_PIN =   A7;

// I2C address
const byte I2C_ADDR               = 0x16;

// Commands
const byte CMD_MOVE               = 0x4D;  // 'M'
const byte CMD_READ               = 0x52;  // 'R'

// Motors
const word MOTOR_MAX_MM           = 1400;
const word MOTOR_MM2STEPS         =   25;
const word MOTOR_SPEED            =  200;
const word MOTOR_MAX_STEPS        = MOTOR_MAX_MM * MOTOR_MM2STEPS;
const word MOTOR_MM2STEPS_2       = MOTOR_MM2STEPS / 2;

// Other
const word LOOP_DELAY             =   10;


// ****************************************************************************
// Structures
// ****************************************************************************

struct State {
  volatile unsigned long vert;
  byte pan, tilt, flags, bat1Voltage, bat2Voltage;
};

struct Target {
  unsigned long vert;
  byte pan, tilt;
};


// ****************************************************************************
// Variables
// ****************************************************************************

DualMC33926MotorShield motorVert;
Servo motorPan, motorTilt;
byte cmd = CMD_READ;
State state = { 0, 0, 0, 0, 0, 0 };
Target target = { 0, 0, 0 };


// ****************************************************************************
// Main
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
}

// Loop method
void loop() {
  updateMotor(motorVert, state.vert, target.vert);
  updateServo(motorPan , state.pan , target.pan );
  updateServo(motorTilt, state.tilt, target.tilt);
  dump();
  delay(LOOP_DELAY);
}


// ****************************************************************************
// Motors
// ****************************************************************************

// Updates the movement of a motor
void updateMotor(DualMC33926MotorShield& motor,
                 volatile unsigned long& current, unsigned long target) {
  if      (current < target) motor.setM1Speed(-MOTOR_SPEED); // Up
  else if (current > target) motor.setM1Speed(+MOTOR_SPEED); // Down
  else                       motor.setM1Speed(0);            // Stop
}

// Updates the movement of a servo
void updateServo(Servo& motor, byte& current, byte target) {
  current = motor.read();
  if      (current < target) motor.write(current + 1); // Up
  else if (current > target) motor.write(current - 1); // Down
}

// Reads the encoder values
void readEncoder() {
  if (digitalRead(ENCODER_A_PIN) != digitalRead(ENCODER_B_PIN)) {
    if (state.vert < MOTOR_MAX_STEPS) state.vert += 1; // Up
  } else {
    if (state.vert > 0)               state.vert -= 1; // Down
  }
}


// ****************************************************************************
// Communication
// ****************************************************************************

// Receives data
void receiveEvent(int howMany) {
  if (!Wire.available()) {
    Serial.println("ERROR: No data to read");
    return;
  }
  cmd = Wire.read();
  switch (cmd) {
    case CMD_READ: {
      return;
    }
    case CMD_MOVE: {
      // - PAYLOAD: | VERT_H | VERT_L | PAN | TILT |
      if (Wire.available() != 4) {
        Serial.println("ERROR: Wrong number of bytes");
        return;
      }
      word mmVert = (Wire.read() << 8) | Wire.read();
      target.vert = mm2steps(mmVert);
      target.pan  = Wire.read();
      target.tilt = Wire.read();
      return;
    }
  }
  Serial.println("ERROR: Unknown command");
}

// Receives a request for data
void requestEvent() {
  // Check read state
  if (cmd != CMD_READ) {
    Serial.println("ERROR: Read not requested");
    return;
  }
  // Serialize
  word mmVert = steps2mm(state.vert);
  byte msg[] = {
    (byte)(mmVert >> 8), (byte)mmVert, state.pan, state.tilt,
    state.flags, state.bat1Voltage, state.bat2Voltage
  };
  // Send
  Wire.write(msg, 7);
}


// ****************************************************************************
// Helpers
// ****************************************************************************

// Converts mm to steps
unsigned long mm2steps(word mm) {
  return mm * MOTOR_MM2STEPS;
}

// Converts steps to mm
word steps2mm(unsigned long steps) {
  return (steps + MOTOR_MM2STEPS_2) / MOTOR_MM2STEPS;
}

// Prints the current conditions
void dump() {
  Serial.print((char)cmd);
  Serial.print(" | ");
  Serial.print(steps2mm(target.vert));
  Serial.print(" mm, ");
  Serial.print(target.pan);
  Serial.print("º, ");
  Serial.print(target.tilt);
  Serial.print("º");
  Serial.print(" | ");
  Serial.print(steps2mm(state.vert));
  Serial.print(" mm, ");
  Serial.print(state.pan);
  Serial.print("º, ");
  Serial.print(state.tilt);
  Serial.print("º");
  Serial.print("        \r");
}
