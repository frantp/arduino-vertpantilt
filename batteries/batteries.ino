/**
  batteries.ino: Manage battery connection, LED updates and state transmision.

  The LEDs have the following meaning:
  - Green color: Battery is fully charged
  - Amber color: Battery is half charged
  - Red color:   Battery is uncharged
  - Blink state: Battery is in use
  - Fixed state: Battery is not in use

  The state is sent through the serial interface using two possible methods:
  - The following binary dataframe (5 bytes):
      +--------+-------+------+------+-----+
      | HEADER | FLAGS | BT1V | BT2V | CHK |
      +--------+-------+------+------+-----+
      - HEADER:    Constant value: 0xFE
      - FLAGS:     Bit 2: battery 1 connected, bit 0: battery 2 connected
      - BT1V/BT2V: Battery 1/2 voltage
      - CHK:       Checksum
  - The following text dataframe (14 characters):
    @A<C1><V1>$@B<C2><V2>$
      - C1/C2: Battery 1/2 connected (1 character)
      - V1/V2: Battery 1/2 voltage, in mV (3 characters)
*/


// ****************************************************************************
// Constants
// ****************************************************************************

const bool BINARY_MODE          =  true;

// Pins
const byte BT1_RELAY_ON_PIN     =     2;
const byte BT1_RELAY_OFF_PIN    =     3;
const byte BT2_RELAY_ON_PIN     =     6;
const byte BT2_RELAY_OFF_PIN    =     7;
const byte BT1_GREEN_LED_PIN    =     4;
const byte BT1_RED_LED_PIN      =     5;
const byte BT2_GREEN_LED_PIN    =     8;
const byte BT2_RED_LED_PIN      =     9;
const byte BT1_VOLTAGE_PIN      =    A0;
const byte BT2_VOLTAGE_PIN      =    A1;
const byte BT1_STATE_PIN        =    A6;
const byte BT2_STATE_PIN        =    A7;

// Batteries
const word BATTERY_CONNECTED_TH =   200;
const word BATTERY_HIGH_TH      =   120;  // dV
const word BATTERY_MEDIUM_TH    =   115;  // dV
const word BATTERY_LOW_TH       =   110;  // dV

// Flags
const byte FLAG_BT1STATE        =     0;
const byte FLAG_BT2STATE        =     2;

// Message
const byte MSG_HEADER           =  0xFE;

// Other
const word LOOP_DELAY           =   500;  // ms


// ****************************************************************************
// Structures
// ****************************************************************************

struct State {
  byte bt1Voltage, bt2Voltage;  // dV
  bool bt1Connected, bt2Connected;
};


// ****************************************************************************
// Variables
// ****************************************************************************

State state = { 0, 0, 0, 0 };
bool blink = false;


// ****************************************************************************
// Main
// ****************************************************************************

// ----------------------------------------------------------------------------
// Setup method
void setup() 
{
  // Pins
  pinMode(BT1_RELAY_ON_PIN , OUTPUT);
  pinMode(BT1_RELAY_OFF_PIN, OUTPUT);
  pinMode(BT2_RELAY_ON_PIN , OUTPUT);
  pinMode(BT2_RELAY_OFF_PIN, OUTPUT);
  pinMode(BT1_GREEN_LED_PIN, OUTPUT);
  pinMode(BT1_RED_LED_PIN  , OUTPUT);
  pinMode(BT2_GREEN_LED_PIN, OUTPUT);
  pinMode(BT2_RED_LED_PIN  , OUTPUT);
  digitalWrite(BT1_RELAY_ON_PIN , HIGH);
  digitalWrite(BT1_RELAY_OFF_PIN, HIGH);
  digitalWrite(BT2_RELAY_ON_PIN , HIGH);
  digitalWrite(BT2_RELAY_OFF_PIN, HIGH);
  digitalWrite(BT1_GREEN_LED_PIN, LOW );  
  digitalWrite(BT1_RED_LED_PIN  , LOW );
  digitalWrite(BT2_GREEN_LED_PIN, LOW );
  digitalWrite(BT2_RED_LED_PIN  , LOW );

  // Serial
  Serial.begin(9600);

  delay(2000);
}

// ----------------------------------------------------------------------------
// Loop method
void loop() 
{
  // Get state
  state.bt1Voltage   = (word)analogRead(BT1_VOLTAGE_PIN) * 10 / 73;
  state.bt2Voltage   = (word)analogRead(BT2_VOLTAGE_PIN) * 10 / 73;
  state.bt1Connected = (word)analogRead(BT1_STATE_PIN) >= BATTERY_CONNECTED_TH;
  state.bt2Connected = (word)analogRead(BT2_STATE_PIN) >= BATTERY_CONNECTED_TH;

  // Update battery connections
  // If none connected, connect the first one that is charged
  if (!state.bt1Connected && !state.bt2Connected) {
    if        (state.bt1Voltage >= BATTERY_LOW_TH) {
      switchBattery(BT1_RELAY_ON_PIN, BT2_RELAY_OFF_PIN);
    } else if (state.bt1Voltage >= BATTERY_LOW_TH) {
      switchBattery(BT2_RELAY_ON_PIN, BT1_RELAY_OFF_PIN);
    }
  // If battery 1 low, switch to battery 2
  } else if (state.bt1Connected
          && state.bt1Voltage <  BATTERY_LOW_TH
          && state.bt2Voltage >= BATTERY_LOW_TH) {
    switchBattery(BT2_RELAY_ON_PIN, BT1_RELAY_OFF_PIN);
  // If battery 2 low, switch to battery 1
  } else if (state.bt2Connected
          && state.bt2Voltage <  BATTERY_LOW_TH
          && state.bt1Voltage >= BATTERY_LOW_TH) {
    switchBattery(BT1_RELAY_ON_PIN, BT2_RELAY_OFF_PIN);
  }

  // Update LED state
  blink = !blink;
  updateLeds(state.bt1Connected, state.bt1Voltage,
             BT1_GREEN_LED_PIN, BT1_RED_LED_PIN);
  updateLeds(state.bt2Connected, state.bt2Voltage,
             BT2_GREEN_LED_PIN, BT2_RED_LED_PIN);

  // Send state
  if (BINARY_MODE) {
    // - PAYLOAD: | HEADER | FLAGS | BT1V | BT2V | CHK |
    byte msg[] = {
      MSG_HEADER,
      (byte)(state.bt1Connected << FLAG_BT2STATE |
             state.bt2Connected << FLAG_BT1STATE),
      state.bt1Voltage, state.bt2Voltage, 0
    };
    for (byte i = 1; i < 4; i++) msg[4] += msg[i];
    msg[4] = 0xFF - msg[4] + 1;
    for (int i = 0; i < 5; i++) {
      Serial.write(msg[i]);
      delay(2);
    }
  } else {
    char serialOutBuffer[15];
    sprintf(serialOutBuffer, "@A%1d%03d$@B%1d%03d$",
      state.bt1Connected, state.bt1Voltage,
      state.bt2Connected, state.bt2Voltage);
    for (int i = 0; i < 14; i++) {
      Serial.print(serialOutBuffer[i]);
      delay(2);
    }
    Serial.println();
  }

  delay(LOOP_DELAY);
}


// ****************************************************************************
// Helpers
// ****************************************************************************

// ----------------------------------------------------------------------------
// Battery connection handling
void switchBattery(byte onPin, byte offPin) {
  digitalWrite(onPin , LOW );
  delay(200);
  digitalWrite(onPin , HIGH);
  delay(200);
  digitalWrite(offPin, LOW );
  delay(200);
  digitalWrite(offPin, HIGH);
}


// ----------------------------------------------------------------------------
// LED handling
void updateLeds(bool connected, word voltage, byte greenPin, byte redPin) {
  const auto value = connected || blink ? HIGH : LOW;
  digitalWrite(greenPin, voltage >= BATTERY_MEDIUM_TH ? value : LOW);
  digitalWrite(redPin  , voltage <  BATTERY_HIGH_TH   ? value : LOW);
}
