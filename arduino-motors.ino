/**
  arduino-motors.ino: Manage motors for the movement in three axes: vertical,
    pan and tilt.

  The system holds two main structures, one with the current state (including
  position) and other with the target position. At each loop iteration, the
  motors are moved, if necessary, in the direction of the target position, one
  step at a time. This way, the three motors can be moved at the same time,
  interleaving their steps. Moreover, the movement is non-blocking and is
  decoupled from the communication, with the possibility of updating the target
  position even before the motors reach the previous target.

  There are two channels for communication:

  Serial
  ================
  String dataframes of the form '@<C><N>$' to update the target position of
  each motor individually.
    - <C> is the single-character movement command:
        'V' for vertical, 'P' for pan, 'T' for tilt
    - <N> is a multiple-character integer with the target position:
        vertical in mm, pan and tilt in degrees

  The state of the system is constantly updated in a single constant-width line
  with zero-padded numbers.

  I2C
  ================
  Two possible byte dataframes:
    - Move (5 bytes): Updates the target position for the three motors at the
      same time:
        -------------------------------------------
        | CMD_MOVE | VERT_H | VERT_L | PAN | TILT |
        -------------------------------------------
    - Read (1 byte): Requests the current state of the system:
        ------------
        | CMD_READ |
        ------------
      The return dataframe is 7 bytes long:
        --------------------------------------------------------------
        | VERT_H | VERT_L | PAN | TILT | FLAGS | BAT1VOLT | BAT2VOLT |
        --------------------------------------------------------------
*/

#include <Servo.h>
#include <Wire.h>


// ****************************************************************************
// Constants
// ****************************************************************************

// Pins
const byte ENCODER_A_PIN          =    2;
const byte ENCODER_B_PIN          =    3;
const byte ENCODER_INT_PIN        =    2;
const byte MOTOR_PWM_PIN          =    6;
const byte MOTOR_DIR_PIN          =    7;
const byte MOTOR_FBK_PIN          =   A0;
const byte MOTOR_ND2_PIN          =    4;
const byte MOTOR_NSF_PIN          =   12;
const byte SERVO_PAN_PIN          =   11;
const byte SERVO_PAN_FBK_PIN      =   A2;
const byte SERVO_TILT_PIN         =   10;
const byte SERVO_TILT_FBK_PIN     =   A1;
const byte LOWER_LIMIT_SWITCH_PIN =   A6;
const byte UPPER_LIMIT_SWITCH_PIN =   A7;

// Communication
const byte I2C_ADDR               = 0x16;
const char SERIAL_STR_MARKER      = '@';
const char SERIAL_END_MARKER      = '$';

// Commands
const byte CMD_MOVE               = 0x4D;  // 'M'
const byte CMD_READ               = 0x52;  // 'R'

// Motors
const word MOTOR_MM_STEPS         =   25;
const word MOTOR_MM_STEPS_2       = MOTOR_MM_STEPS / 2;
const byte MOTOR_MAX_SPEED        =  127;
const word MOTOR_SPEED_TH         =   10;  // mm

// Other
const word LIMIT_SWITCH_THRESHOLD =  400;
const word LOOP_DELAY             =    5;  // ms


// ****************************************************************************
// Structures
// ****************************************************************************

struct State {
  word vert;
  byte pan, tilt, flags, bat1Voltage, bat2Voltage;
};

struct Target {
  word vert;
  byte pan, tilt;
};


// ****************************************************************************
// Variables
// ****************************************************************************

// Motors
Servo motorPan, motorTilt;
volatile unsigned long vert = 0;
// Context
byte cmd = CMD_READ;
State state = { 0, 0, 0, 0, 0, 0 };
Target target = { 0, 0, 0 };
// Serial
char serialBuffer[8];
byte serialIdx = 0;
char serialStarted = false;


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
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_FBK_PIN, INPUT);
  pinMode(MOTOR_ND2_PIN, OUTPUT);
  digitalWrite(MOTOR_ND2_PIN, HIGH);
  pinMode(MOTOR_NSF_PIN, INPUT);

  // Interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_INT_PIN),
    readEncoder, FALLING);

  // Servos
  motorPan.attach(SERVO_PAN_PIN);  
  motorTilt.attach(SERVO_TILT_PIN);

  // Wire and serial
  Wire.begin(I2C_ADDR);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Serial.begin(9600);

  // Move motors to position that is read from feedback
  const byte panPos  = analogRead(SERVO_PAN_FBK_PIN ) * 10 / 42;
  const byte tiltPos = analogRead(SERVO_TILT_FBK_PIN) * 10 / 42;
  motorPan.write(panPos);
  motorTilt.write(tiltPos);
}

// Loop method
void loop() {
  readSerial();
  noInterrupts();
  state.vert = (vert + MOTOR_MM_STEPS_2) / MOTOR_MM_STEPS;
  interrupts();
  updateMotor(state.vert, target.vert);
  state.pan  =       updateServo(motorPan,        target.pan );
  state.tilt = 180 - updateServo(motorTilt, 180 - target.tilt);
  dump();
  delay(LOOP_DELAY);
}


// ****************************************************************************
// Motors
// ****************************************************************************

// Updates the movement of a motor
void updateMotor(word current, word target) {
  char dir = 0;                        // Stop
  if      (target == 0)      dir = -1; // Down
  else if (current < target) dir = +1; // Up
  else if (current > target) dir = -1; // Down
  // Limit switches
  if (analogRead(LOWER_LIMIT_SWITCH_PIN) < LIMIT_SWITCH_THRESHOLD) {
    if (dir < 0) dir = 0; // Stop down motion
  }
  if (analogRead(UPPER_LIMIT_SWITCH_PIN) < LIMIT_SWITCH_THRESHOLD) {
    if (dir > 0) dir = 0; // Stop up motion
  }
  // Adjust speed
  const unsigned long diff = (target - current) * motorDirection;
  byte speed;
  if      (diff == 0)              speed = 0;
  else if (diff < MOTOR_SPEED_LTH) speed = MOTOR_MAX_SPEED >> 2;
  else if (diff < MOTOR_SPEED_HTH) speed = MOTOR_MAX_SPEED >> 1;
  else                             speed = MOTOR_MAX_SPEED;
  // Update
  analogWrite(MOTOR_PWM_PIN, speed);
  digitalWrite(MOTOR_DIR_PIN, dir > 0 ? HIGH : LOW);
}

// Updates the movement of a servo
byte updateServo(Servo& motor, byte target) {
  byte current = motor.read();
  if      (current < target) motor.write(current + 1); // Up
  else if (current > target) motor.write(current - 1); // Down
  return current;
}

// Reads the encoder values
void readEncoder() {
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
                  vert++; // Up
  } else {
    if (vert > 0) vert--; // Down
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
      target.vert = (Wire.read() << 8) | Wire.read();
      target.pan  = Wire.read();
      target.tilt = Wire.read();
      return;
    }
  }
  Serial.print("ERROR: Unknown command ");
  Serial.println(cmd, HEX);
}

// Receives a request for data
void requestEvent() {
  // Check read state
  if (cmd != CMD_READ) {
    Serial.println("ERROR: Read not requested");
    return;
  }
  // Serialize
  // - PAYLOAD: | VERT_H | VERT_L | PAN | TILT | FLAGS | BAT1VOLT | BAT2VOLT |
  const byte msg[] = {
    (byte)(state.vert >> 8), (byte)state.vert, state.pan, state.tilt,
    state.flags, state.bat1Voltage, state.bat2Voltage
  };
  // Send
  Wire.write(msg, 7);
}

// Reads input from serial
void readSerial() {
  while (Serial.available()) {
    const char ch = Serial.read();
    if (ch == SERIAL_STR_MARKER) {
      serialStarted = true;
      serialIdx = 0;
    } else if (serialStarted) {
      if (ch == SERIAL_END_MARKER) {
        serialBuffer[serialIdx++] = '\0';
        processSerialCommand(serialBuffer);
        serialStarted = false;
      } else {
        serialBuffer[serialIdx++] = ch;
      }
    }
    if (serialIdx == 7) {
      Serial.print("ERROR: Command too long");
      serialStarted = false;
    }
  }
}

// Process a command sent through the serial interface
void processSerialCommand(const char* cmd) {
  word value = atoi(cmd + 1);
  switch (cmd[0]) {
    case 'V': target.vert = value; return;
    case 'P': target.pan  = value; return;
    case 'T': target.tilt = value; return;
  }
  Serial.print("ERROR: Unknown command ");
  Serial.println(cmd[0]);
}


// ****************************************************************************
// Helpers
// ****************************************************************************

// Prints the current conditions
void dump() {
  char buffer[128];
  sprintf(buffer,
    "%c | "
    "V: %04d mm, P: %03dº T: %03dº | "
    "V: %04d mm, P: %03dº T: %03dº | "
    "F: %02X, B1: %02d.%01d mV, B2: %02d.%01d mV",
    (const char)cmd,
    target.vert, target.pan, target.tilt,
    state.vert , state.pan , state.tilt ,
    state.flags,
    state.bat1Voltage / 10, state.bat1Voltage % 10,
    state.bat2Voltage / 10, state.bat2Voltage % 10);
  Serial.println(buffer);
}
