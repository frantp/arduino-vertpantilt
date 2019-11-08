/**
  vertpantilt.ino: Manage motors for the movement in three axes: vertical, pan
  and tilt.

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

  Serial output is non-blocking, writing data one character at a time. This
  reduces the extra delay in the main loop to a minimum, lessening the
  interference with the movement of the servos.

  I2C
  ================
  Two possible binary dataframes (big-endian):
    - Move (6 bytes): Updates the target position for the three motors at the
      same time:
        +----------+----+----+-----+------+-----+
        | CMD_MOVE |  VERT   | PAN | TILT | CHK |
        +----------+----+----+-----+------+-----+
    - Read (1 byte): Requests the current state of the system:
        +----------+
        | CMD_READ |
        +----------+
      The return dataframe is 8 bytes long:
        +----+----+-----+------+-------+------+------+-----+
        |  VERT   | PAN | TILT | FLAGS | BT1V | BT2V | CHK |
        +----+----+-----+------+-------+------+------+-----+
      The "flags" byte has the following contents:
        +-----+-----+-----+-----+-----+-----+-----+-----+
        | MFT | MST | ULS | LLS |    BT1    |    BT2    |
        +-----+-----+-----+-----+-----+-----+-----+-----+
        - MFT:        Motor fault
        - MST:        Motor stopped
        - ULS/LLS:    Upper/Lower limit switch pressed
        - BT1V/BT2V:  Battery 1/2 voltage
*/

#include <Servo.h>
#include <Wire.h>


// ****************************************************************************
// Constants
// ****************************************************************************

const bool BINARY_MODE            =  true;

// Pins
const byte ENCODER_A_PIN          =     2;
const byte ENCODER_B_PIN          =     3;
const byte ENCODER_INT_PIN        =     2;
const byte MOTOR_PWM_PIN          =     6;
const byte MOTOR_DIR_PIN          =     7;
const byte MOTOR_FBK_PIN          =    A0;
const byte MOTOR_ND2_PIN          =     4;
const byte MOTOR_NSF_PIN          =    12;
const byte SERVO_PAN_PIN          =    11;
const byte SERVO_PAN_FBK_PIN      =    A2;
const byte SERVO_TILT_PIN         =    10;
const byte SERVO_TILT_FBK_PIN     =    A1;
const byte LOWER_LIMIT_SWITCH_PIN =    A6;
const byte UPPER_LIMIT_SWITCH_PIN =    A7;
const byte WIRELESS_PIN           =     9;

// Communication
const byte I2C_ADDR               =  0x16;
const char SERIAL_BGN_MARKER      =   '@';
const char SERIAL_END_MARKER      =   '$';

// Commands
const byte CMD_MOVE               =  0x4D;  // 'M'
const byte CMD_READ               =  0x52;  // 'R'

// Motors
const word MOTOR_MM_STEPS         =    25;
const byte MOTOR_SPEED            =   127;
const byte MOTOR_FBK_TH           =   110;
const word LIMIT_SWITCH_TH        =   400;
const word MAX_VERT_STEPS         = 0xFFFF - MOTOR_MM_STEPS;

// Flags
const byte FLAG_BT1STATE          =     0;
const byte FLAG_BT2STATE          =     2;
const byte FLAG_LOWER_LIMIT       =     4;
const byte FLAG_UPPER_LIMIT       =     5;
const byte FLAG_MOTOR_STOP        =     6;
const byte FLAG_MOTOR_FAULT       =     7;

// Other
const word LOOP_DELAY             =    10;  // ms


// ****************************************************************************
// Structures
// ****************************************************************************

struct State {
  word vert;
  byte pan, tilt, flags, bt1Voltage, bt2Voltage;
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

// Context
volatile byte cmd = CMD_READ;
volatile word vert_steps = 0;
volatile State state = { 0, 0, 0, 0, 0, 0 };
volatile Target target = { 0, 0, 0 };

// Serial
char serialInBuffer[8];
byte serialInIdx = 0;
char serialInStarted = false;
char serialOutBuffer[128];
byte serialOutIdx = 0;


// ****************************************************************************
// Main
// ****************************************************************************

// ----------------------------------------------------------------------------
// Setup method
void setup() {
  // Pins
  pinMode(ENCODER_A_PIN, INPUT );
  pinMode(ENCODER_B_PIN, INPUT );
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_FBK_PIN, INPUT );
  pinMode(MOTOR_ND2_PIN, OUTPUT);
  pinMode(MOTOR_NSF_PIN, INPUT );
  pinMode(WIRELESS_PIN , OUTPUT);
  digitalWrite(ENCODER_A_PIN, HIGH);
  digitalWrite(ENCODER_B_PIN, HIGH);
  digitalWrite(MOTOR_ND2_PIN, HIGH);
  digitalWrite(WIRELESS_PIN , HIGH);

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

  delay(2000);

  // Move motors to position that is read from feedback
  const byte panPos  = analogRead(SERVO_PAN_FBK_PIN ) * 10 / 42;
  const byte tiltPos = analogRead(SERVO_TILT_FBK_PIN) * 10 / 42;
  motorPan.write(panPos);
  motorTilt.write(tiltPos);
}

// ----------------------------------------------------------------------------
// Loop method
void loop() {
  if (BINARY_MODE) {
    readBatteryState();
  } else {
    readSerial();
  }
  noInterrupts();
  state.vert = (vert_steps + MOTOR_MM_STEPS / 2) / MOTOR_MM_STEPS;
  word state_vert  = state.vert;
  word target_vert = target.vert;
  interrupts();
  updateMotor(state_vert, target_vert);
  state.pan  =       updateServo(motorPan,        target.pan );
  state.tilt = 180 - updateServo(motorTilt, 180 - target.tilt);
  writeSerial();
  delay(LOOP_DELAY);
}


// ****************************************************************************
// Motors
// ****************************************************************************

// ----------------------------------------------------------------------------
// Updates the movement of a motor
void updateMotor(word current, word target) {
  // Compute direction
  char dir = 0;                        // Stop
  if      (target == 0)      dir = -1; // Down
  else if (current < target) dir = +1; // Up
  else if (current > target) dir = -1; // Down

  // Handle corner cases
  // - Motor fault detected
  if (!digitalRead(MOTOR_NSF_PIN)) {
    dir = 0; // Stop motion
    bitSet  (state.flags, FLAG_MOTOR_FAULT);
  } else {
    bitClear(state.flags, FLAG_MOTOR_FAULT);
  }
  // - Motor feedback current too high
  if (analogRead(MOTOR_FBK_PIN) > MOTOR_FBK_TH) {
    dir = 0; // Stop motion
    bitSet  (state.flags, FLAG_MOTOR_STOP);
  } else {
    bitClear(state.flags, FLAG_MOTOR_STOP);
  }
  // - Lower limit switch pressed
  if (analogRead(LOWER_LIMIT_SWITCH_PIN) < LIMIT_SWITCH_TH) {
    if (dir < 0) dir = 0; // Stop down motion
    noInterrupts();
    vert_steps = 0;
    interrupts();
    bitSet  (state.flags, FLAG_LOWER_LIMIT);
  } else {
    bitClear(state.flags, FLAG_LOWER_LIMIT);
  }
  // - Upper limit switch pressed
  if (analogRead(UPPER_LIMIT_SWITCH_PIN) < LIMIT_SWITCH_TH) {
    if (dir > 0) dir = 0; // Stop up motion
    bitSet  (state.flags, FLAG_UPPER_LIMIT);
  } else {
    bitClear(state.flags, FLAG_UPPER_LIMIT);
  }

  // Update
  analogWrite(MOTOR_PWM_PIN, dir != 0 ? MOTOR_SPEED : 0);
  digitalWrite(MOTOR_DIR_PIN, dir > 0 ? HIGH : LOW);
}

// ----------------------------------------------------------------------------
// Updates the movement of a servo
byte updateServo(Servo& motor, byte target) {
  byte current = motor.read();
  if      (current < target) motor.write(current + 1); // Up
  else if (current > target) motor.write(current - 1); // Down
  return current;
}

// ----------------------------------------------------------------------------
// Reads the encoder values
void readEncoder() {
  if (digitalRead(ENCODER_A_PIN) == digitalRead(ENCODER_B_PIN)) {
    if (vert_steps < MAX_VERT_STEPS) vert_steps++; // Up
  } else {
    if (vert_steps > 0)              vert_steps--; // Down
  }
}


// ****************************************************************************
// Communication
// ****************************************************************************

// ----------------------------------------------------------------------------
// Receives data
void receiveEvent(int numBytes) {
  // Read command
  if (numBytes < 1) {
    Serial.println("\nERROR: No data to read");
    return;
  }
  cmd = Wire.read();

  // Update context based on command
  byte read = 1;
  switch (cmd) {
    case CMD_READ: {
    } break;

    case CMD_MOVE: {
      // - PAYLOAD: | VERT_H | VERT_L | PAN | TILT | CHK |
      if (Wire.available() < 5) {
        Serial.println("\nERROR: Wrong number of bytes");
        break;
      }
      const byte vert_h = Wire.read();
      const byte vert_l = Wire.read();
      const byte pan    = Wire.read();
      const byte tilt   = Wire.read();
      const byte chk    = Wire.read();
      read += 5;
      const byte sum = vert_h + vert_l + pan + tilt + chk;
      if (sum != 0) {
        Serial.println("\nERROR: Incorrect checksum");
        break;
      }
      target.vert = (vert_h << 8) | vert_l;
      target.pan  = pan;
      target.tilt = tilt;
    } break;

    default: {
      Serial.print("\nERROR: Unknown command ");
      Serial.println(cmd, HEX);
    } break;
  }

  // Clear buffer
  for (byte i = 0; i < numBytes - read; i++) Wire.read();
}

// ----------------------------------------------------------------------------
// Receives a request for data
void requestEvent() {
  // Check read state
  if (cmd != CMD_READ) {
    Serial.println("\nERROR: Read not requested");
    return;
  }

  // Serialize
  // - PAYLOAD: | VERT_H | VERT_L | PAN | TILT | FLAGS | BT1V | BT2V | CHK |
  byte msg[] = {
    (byte)(state.vert >> 8), (byte)state.vert, state.pan, state.tilt,
    state.flags, state.bt1Voltage, state.bt2Voltage, 0
  };
  for (byte i = 0; i < 7; i++) msg[7] += msg[i];
  msg[7] = 0xFF - msg[7] + 1;

  // Send
  Wire.write(msg, 8);
}

// ----------------------------------------------------------------------------
// Reads the state of the batteries from the wireless serial module
void readBatteryState() {
  // - PAYLOAD: | HEADER | BT1V | BT2V | CHK |
  const byte MSG_HEADER = 0xFE;
  while (Serial.available() && Serial.read() != MSG_HEADER);
  if (Serial.available() < 5) return;
  const byte flags = Serial.read();
  const byte bt1v  = Serial.read();
  const byte bt2v  = Serial.read();
  const byte chk   = Serial.read();
  const byte sum = flags + bt1v + bt2v + chk;
  if (sum != 0) {
    Serial.println("\nERROR: Incorrect checksum");
    return;
  }
  if (flags & FLAG_BT1STATE) bitSet  (state.flags, FLAG_BT1STATE);
  else                       bitClear(state.flags, FLAG_BT1STATE);
  if (flags & FLAG_BT2STATE) bitSet  (state.flags, FLAG_BT2STATE);
  else                       bitClear(state.flags, FLAG_BT2STATE);
  state.bt1Voltage = bt1v;
  state.bt2Voltage = bt2v;
}

// ----------------------------------------------------------------------------
// Reads input from serial
void readSerial() {
  while (Serial.available()) {
    // Read character
    const char ch = Serial.read();
    if (ch == SERIAL_BGN_MARKER) {
      // - Start
      serialInStarted = true;
      serialInIdx = 0;
    } else if (serialInStarted) {
      if (ch != SERIAL_END_MARKER) {
        // - Middle
        serialInBuffer[serialInIdx++] = ch;
      } else {
        // - End
        serialInBuffer[serialInIdx] = '\0';
        processSerialCommand(serialInBuffer);
        serialInStarted = false;
      }
    }

    // Check command length
    if (serialInIdx == 8) {
      Serial.print("\nERROR: Command too long");
      serialInStarted = false;
    }
  }
}

// ----------------------------------------------------------------------------
// Process a command sent through the serial interface
void processSerialCommand(const char* cmd) {
  const unsigned long TH = 1000;
  const unsigned long value = atol(cmd + 1);
  switch (cmd[0]) {
    case 'V': target.vert = value; return;
    case 'P': target.pan  = value; return;
    case 'T': target.tilt = value; return;
    case 'A':
      state.bt1Voltage = value % TH;
      if (value >= TH) bitSet  (state.flags, FLAG_BT1STATE);
      else             bitClear(state.flags, FLAG_BT1STATE);
      return;
    case 'B':
      state.bt2Voltage = value % TH;
      if (value >= TH) bitSet  (state.flags, FLAG_BT2STATE);
      else             bitClear(state.flags, FLAG_BT2STATE);
      return;
  }
  Serial.print("\nERROR: Unknown command ");
  Serial.println(cmd[0]);
}

// ----------------------------------------------------------------------------
// Prints the current conditions
void writeSerial() {
  if (serialOutIdx == 0) {
    // Build string
    sprintf(serialOutBuffer,
      "%c | "
      "V: %4d mm, P: %3dº T: %3dº | "
      "V: %4d mm, P: %3dº T: %3dº | "
      "F: %02X, B1: %2d.%01d V, B2: %2d.%01d V\n",
      (const char)cmd,
      target.vert, target.pan, target.tilt,
      state.vert , state.pan , state.tilt ,
      state.flags,
      state.bt1Voltage / 10, state.bt1Voltage % 10,
      state.bt2Voltage / 10, state.bt2Voltage % 10);
  }

  // Print one character at a time
  Serial.print(serialOutBuffer[serialOutIdx++]);
  if (serialOutIdx == 128 || serialOutBuffer[serialOutIdx] == '\0') {
    serialOutIdx = 0;
  }
}
