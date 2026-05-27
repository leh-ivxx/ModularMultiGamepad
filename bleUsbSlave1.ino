#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BleGamepad.h>

//================================================
// PIN DEFINITIONS (6 BUTTONS + 2 ANALOG + ENCODER)
//================================================

#define D0 16
#define D1 17
#define D2 18
#define D3 19
#define D4 14
#define D5 27
#define A0 39
#define A1 36
#define ENC_A 4
#define ENC_B 5

//================================================
// CONFIGURATION CONSTANTS
//================================================

#define ANALOG_RES 4095
#define ANALOG_CENTER 2048
#define AXIS_MIN -32767
#define AXIS_MAX 32767
#define MAX_MAP 8
#define MAX_AXES 8
#define MAX_BUTTONS 32
#define MAX_DIGITAL_INPUTS 6
#define ESP_NOW_SEND_INTERVAL 5  // milliseconds
#define DEVICE_NAME_SIZE 32
#define MAC_ADDRESS_SIZE 6
#define SERIAL_BUFFER_SIZE 64
#define MAX_ENCODER_VALUE 131072

//================================================
// DEVICE MODES
//================================================

enum Mode {
  MODE_BLE_ONLY,
  MODE_ESPNOW_ONLY,
  MODE_HYBRID
};

enum CommandType {
  CMD_MAP,
  CMD_DISABLE,
  CMD_ENABLE,
  CMD_LIST,
  CMD_RESET,
  CMD_SAVE,
  CMD_STATUS,
  CMD_NAME,
  CMD_DEADZONE,
  CMD_INVERT,
  CMD_SCALE,
  CMD_ENCODER_AXIS,
  CMD_ENCODER_STEP,
  CMD_UNKNOWN
};

//================================================
// CONFIGURATION STRUCTURES
//================================================

struct InputMap {
  uint8_t type;
  uint8_t source;
  uint8_t target;
  bool enabled;
};

struct AnalogConfig {
  int deadzone;
  bool invert;
  int outMin;
  int outMax;
};

struct EncoderConfig {
  uint8_t axis;
  int step;
};

typedef struct {
  char name[DEVICE_NAME_SIZE];
  uint32_t buttons;
  int16_t axis[MAX_AXES];
} GamepadPacket;

//================================================
// GLOBAL STATE
//================================================

Mode deviceMode;
char deviceName[DEVICE_NAME_SIZE] = "EGP-01";
BleGamepad bleGamepad(deviceName, "LEHIVXX", 69);

uint8_t masterAddress[MAC_ADDRESS_SIZE] = {0xCC, 0x8D, 0xA2, 0xEC, 0xDC, 0xAC};

bool digitalInputs[6];
uint16_t analogInputs[2];
uint32_t buttons = 0;
int16_t axis[MAX_AXES];

//================================================
// ENCODER STATE
//================================================

int encoderAccum = 0;
int encoderAccum1 = 0;

int lastEncoderA = 0;
EncoderConfig encoderCfg = {0, 512};  // Default: axis 0, step 1365

//================================================
// INPUT MAPPING & CONFIGURATION
//================================================

InputMap mapTable[MAX_MAP];
AnalogConfig analogCfg[2];
Preferences prefs;
GamepadPacket packet;

//================================================
// TIMING
//================================================

unsigned long lastESPNowSend = 0;

//================================================
// DEFAULT CONFIGURATION
//================================================

void setDefaultMapping() {
  // Map 6 digital inputs (D0-D5) to buttons 0-5
  mapTable[0] = {0, 0, 0, true};
  mapTable[1] = {0, 1, 1, true};
  mapTable[2] = {0, 2, 2, true};
  mapTable[3] = {0, 3, 3, true};
  mapTable[4] = {0, 4, 4, true};
  mapTable[5] = {0, 5, 5, true};
  // Map 2 analog inputs (A0-A1) to axes 0-1
  mapTable[6] = {1, 0, 0, true};
  mapTable[7] = {1, 1, 1, true};
}

void setDefaultAnalogConfig() {
  for (int i = 0; i < 2; i++) {
    analogCfg[i].deadzone = 0;
    analogCfg[i].invert = false;
    analogCfg[i].outMin = AXIS_MIN;
    analogCfg[i].outMax = AXIS_MAX;
  }
}

void setDefaultEncoderConfig() {
  encoderCfg.axis = 0;
  encoderCfg.step = 1365;
}

//================================================
// PERSISTENT STORAGE
//================================================

void loadConfig() {
  prefs.begin("cfg", true);

  if (prefs.isKey("map"))
    prefs.getBytes("map", mapTable, sizeof(mapTable));

  if (prefs.isKey("analog"))
    prefs.getBytes("analog", analogCfg, sizeof(analogCfg));

  if (prefs.isKey("name")) {
    String n = prefs.getString("name");
    n.toCharArray(deviceName, sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
  }

  if (prefs.isKey("encoder"))
    prefs.getBytes("encoder", &encoderCfg, sizeof(encoderCfg));

  prefs.end();
}

void saveConfig() {
  prefs.begin("cfg", false);

  prefs.putBytes("map", mapTable, sizeof(mapTable));
  prefs.putBytes("analog", analogCfg, sizeof(analogCfg));
  prefs.putString("name", deviceName);
  prefs.putBytes("encoder", &encoderCfg, sizeof(encoderCfg));

  prefs.end();
  Serial.println("CONFIG SAVED");
}

void resetConfig() {
  setDefaultMapping();
  setDefaultAnalogConfig();
  setDefaultEncoderConfig();
  strncpy(deviceName, "ESP32_MODULE", sizeof(deviceName) - 1);
  deviceName[sizeof(deviceName) - 1] = '\0';
  Serial.println("CONFIG RESET (not saved)");
}

//================================================
// INPUT READING
//================================================

void readInputs() {
  // Read digital inputs (6 buttons)
  digitalInputs[0] = !digitalRead(D0);
  digitalInputs[1] = !digitalRead(D1);
  digitalInputs[2] = !digitalRead(D2);
  digitalInputs[3] = !digitalRead(D3);
  digitalInputs[4] = !digitalRead(D4);
  digitalInputs[5] = !digitalRead(D5);

  // Read encoder
  int currentA = digitalRead(ENC_A);
  
  if (currentA != lastEncoderA) {
    if (digitalRead(ENC_B) != currentA) {
      encoderAccum1 += encoderCfg.step;
    } else {
      encoderAccum1 -= encoderCfg.step;
    }
    // Clamp to valid axis range
    encoderAccum = constrain(encoderAccum1, AXIS_MIN, AXIS_MAX);
    lastEncoderA = currentA;
  }

  // Read analog inputs
  analogInputs[0] = analogRead(A0);
  analogInputs[1] = analogRead(A1);
}

//================================================
// ANALOG PROCESSING
//================================================

int16_t processAnalog(uint8_t index, int raw) {
  if (index >= 2) return 0;

  AnalogConfig& cfg = analogCfg[index];

  // Apply deadzone
  if (abs(raw - ANALOG_CENTER) < cfg.deadzone) {
    raw = ANALOG_CENTER;
  }

  // Map to output range
  long v = map(raw, 0, ANALOG_RES, cfg.outMin, cfg.outMax);

  // Apply inversion
  if (cfg.invert) {
    v = -v;
  }

  return constrain(v, cfg.outMin, cfg.outMax);
}

//================================================
// INPUT MAPPING
//================================================

void applyMapping() {
  buttons = 0;
  memset(axis, 0, sizeof(axis));

  // Set encoder axis
  if (encoderCfg.axis < MAX_AXES) {
    axis[encoderCfg.axis] = encoderAccum;
  }

  // Process mapped inputs
  for (int i = 0; i < MAX_MAP; i++) {
    InputMap& m = mapTable[i];

    if (!m.enabled) continue;

    // Digital input mapping (6 digital inputs D0-D5)
    if (m.type == 0) {
      if (m.source < MAX_DIGITAL_INPUTS && digitalInputs[m.source]) {
        if (m.target < MAX_BUTTONS) {
          buttons |= (1UL << m.target);
        }
      }
    }
    // Analog input mapping
    else if (m.type == 1) {
      if (m.source < 2 && m.target < MAX_AXES) {
        // Don't overwrite encoder axis
        if (m.target != encoderCfg.axis) {
          axis[m.target] = processAnalog(m.source, analogInputs[m.source]);
        }
      }
    }
  }
}

//================================================
// BLE COMMUNICATION
//================================================

void updateBLE() {
  if (!bleGamepad.isConnected()) return;

  // Update buttons
  for (int i = 0; i < MAX_BUTTONS; i++) {
    if (buttons & (1UL << i)) {
      bleGamepad.press(i + 1);
    } else {
      bleGamepad.release(i + 1);
    }
  }

  // Update axes
  bleGamepad.setAxes(
    axis[0], axis[1], axis[2], axis[3],
    axis[4], axis[5], axis[6], axis[7]
  );
}

//================================================
// ESP-NOW COMMUNICATION
//================================================

void sendESPNOW() {
  strncpy(packet.name, deviceName, sizeof(packet.name) - 1);
  packet.name[sizeof(packet.name) - 1] = '\0';
  packet.buttons = buttons;
  memcpy(packet.axis, axis, sizeof(axis));

  esp_now_send(masterAddress, (uint8_t*)&packet, sizeof(packet));
}

void initESPNow() {
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, masterAddress, MAC_ADDRESS_SIZE);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("ESP-NOW READY");
}

//================================================
// MODE DETECTION
//================================================

void detectMode() {
  bool b1 = !digitalRead(D0);
  bool b2 = !digitalRead(D1);

  if (b1 && !b2) {
    deviceMode = MODE_BLE_ONLY;
  } else if (!b1 && b2) {
    deviceMode = MODE_HYBRID;
  } else {

    
  }deviceMode = MODE_ESPNOW_ONLY;
}

//================================================
// SERIAL COMMAND PARSING
//================================================

int axisNameToIndex(const String& name) {
  String s = name;
  s.toUpperCase();

  if (s == "X") return 0;
  if (s == "Y") return 1;
  if (s == "Z") return 2;
  if (s == "RX") return 3;
  if (s == "RY") return 4;
  if (s == "RZ") return 5;
  if (s == "SL1") return 6;
  if (s == "SL2") return 7;

  return -1;
}

CommandType parseCommandType(const String& cmd) {
  if (cmd.startsWith("MAP")) return CMD_MAP;
  if (cmd.startsWith("DISABLE")) return CMD_DISABLE;
  if (cmd.startsWith("ENABLE")) return CMD_ENABLE;
  if (cmd == "LISTINPUTS") return CMD_LIST;
  if (cmd == "RESETCONFIG") return CMD_RESET;
  if (cmd == "SAVE") return CMD_SAVE;
  if (cmd == "STATUS") return CMD_STATUS;
  if (cmd.startsWith("NAME")) return CMD_NAME;
  if (cmd.startsWith("DEADZONE")) return CMD_DEADZONE;
  if (cmd.startsWith("INVERT")) return CMD_INVERT;
  if (cmd.startsWith("SCALE")) return CMD_SCALE;
  if (cmd.startsWith("ENCODERAXIS")) return CMD_ENCODER_AXIS;
  if (cmd.startsWith("ENCODERSTEP")) return CMD_ENCODER_STEP;
  return CMD_UNKNOWN;
}

void cmdMapDigital(const String& cmd) {
  char in[8] = {0};
  char tgt[8] = {0};

  if (sscanf(cmd.c_str(), "MAP %7s %7s", in, tgt) != 2) {
    Serial.println("INVALID MAP SYNTAX");
    return;
  }

  String input(in);
  String target(tgt);
  input.toUpperCase();
  target.toUpperCase();

  int src = input.substring(1).toInt();
  if (src < 0 || src >= MAX_DIGITAL_INPUTS) {
    Serial.println("INVALID DIGITAL SOURCE (D0-D5)");
    return;
  }

  int btn = target.substring(1).toInt();
  if (btn < 0 || btn >= MAX_BUTTONS) {
    Serial.println("INVALID BUTTON TARGET");
    return;
  }

  mapTable[src].type = 0;
  mapTable[src].source = src;
  mapTable[src].target = btn;
  mapTable[src].enabled = true;

  Serial.println("DIGITAL MAP UPDATED");
}

void cmdMapAnalog(const String& cmd) {
  char in[8] = {0};
  char tgt[8] = {0};

  if (sscanf(cmd.c_str(), "MAP %7s %7s", in, tgt) != 2) {
    Serial.println("INVALID MAP SYNTAX");
    return;
  }

  String input(in);
  String target(tgt);
  input.toUpperCase();
  target.toUpperCase();

  int src = input.substring(1).toInt();
  if (src < 0 || src >= 2) {
    Serial.println("INVALID ANALOG SOURCE (A0-A1)");
    return;
  }

  int axisIndex = axisNameToIndex(target);
  if (axisIndex < 0) {
    Serial.println("INVALID AXIS TARGET");
    return;
  }

  int mapIndex = 6 + src;
  mapTable[mapIndex].type = 1;
  mapTable[mapIndex].source = src;
  mapTable[mapIndex].target = axisIndex;
  mapTable[mapIndex].enabled = true;

  Serial.println("ANALOG MAP UPDATED");
}

void cmdDisableInput(const String& cmd) {
  String input = cmd.substring(8);
  input.toUpperCase();

  if (input[0] == 'D') {
    int idx = input.substring(1).toInt();
    if (idx >= 0 && idx < MAX_DIGITAL_INPUTS) {
      mapTable[idx].enabled = false;
      Serial.println("INPUT DISABLED");
      return;
    }
  } else if (input[0] == 'A') {
    int idx = input.substring(1).toInt();
    if (idx >= 0 && idx < 2) {
      mapTable[6 + idx].enabled = false;
      Serial.println("INPUT DISABLED");
      return;
    }
  }
  Serial.println("INVALID INPUT");
}

void cmdEnableInput(const String& cmd) {
  String input = cmd.substring(7);
  input.toUpperCase();

  if (input[0] == 'D') {
    int idx = input.substring(1).toInt();
    if (idx >= 0 && idx < MAX_DIGITAL_INPUTS) {
      mapTable[idx].enabled = true;
      Serial.println("INPUT ENABLED");
      return;
    }
  } else if (input[0] == 'A') {
    int idx = input.substring(1).toInt();
    if (idx >= 0 && idx < 2) {
      mapTable[6 + idx].enabled = true;
      Serial.println("INPUT ENABLED");
      return;
    }
  }
  Serial.println("INVALID INPUT");
}

void cmdSetName(const String& cmd) {
  String name = cmd.substring(5);
  if (name.length() > 0) {
    name.toCharArray(deviceName, sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
    Serial.print("NAME SET TO: ");
    Serial.println(deviceName);
  }
}

void cmdDeadzone(const String& cmd) {
  int axisNum;
  int value;

  if (sscanf(cmd.c_str(), "DEADZONE A%d %d", &axisNum, &value) != 2) {
    Serial.println("INVALID DEADZONE SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= 2) {
    Serial.println("INVALID ANALOG INDEX (A0-A1)");
    return;
  }

  analogCfg[axisNum].deadzone = value;
  Serial.println("DEADZONE UPDATED");
}

void cmdInvert(const String& cmd) {
  int axisNum;

  if (sscanf(cmd.c_str(), "INVERT A%d", &axisNum) != 1) {
    Serial.println("INVALID INVERT SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= 2) {
    Serial.println("INVALID ANALOG INDEX (A0-A1)");
    return;
  }

  analogCfg[axisNum].invert = !analogCfg[axisNum].invert;
  Serial.println("INVERT TOGGLED");
}

void cmdScale(const String& cmd) {
  int axisNum, minv, maxv;

  if (sscanf(cmd.c_str(), "SCALE A%d %d %d", &axisNum, &minv, &maxv) != 3) {
    Serial.println("INVALID SCALE SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= 2) {
    Serial.println("INVALID ANALOG INDEX (A0-A1)");
    return;
  }

  analogCfg[axisNum].outMin = minv;
  analogCfg[axisNum].outMax = maxv;
  Serial.println("SCALE UPDATED");
}

void cmdEncoderAxis(const String& cmd) {
  int axisNum;

  if (sscanf(cmd.c_str(), "ENCODERAXIS %d", &axisNum) != 1) {
    Serial.println("INVALID ENCODERAXIS SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= MAX_AXES) {
    Serial.print("INVALID AXIS (0-");
    Serial.print(MAX_AXES - 1);
    Serial.println(")");
    return;
  }

  encoderCfg.axis = axisNum;
  Serial.print("ENCODER AXIS SET TO: ");
  Serial.println(axisNum);
}

void cmdEncoderStep(const String& cmd) {
  int step;

  if (sscanf(cmd.c_str(), "ENCODERSTEP %d", &step) != 1) {
    Serial.println("INVALID ENCODERSTEP SYNTAX");
    return;
  }

  if (step <= 0) {
    Serial.println("STEP MUST BE POSITIVE");
    return;
  }

  encoderCfg.step = step;
  Serial.print("ENCODER STEP SET TO: ");
  Serial.println(step);
}

void processSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  switch (parseCommandType(cmd)) {
    case CMD_MAP:
      if (cmd[4] == ' ') {
        char nextChar = (cmd.length() > 5) ? cmd[5] : ' ';
        if (nextChar == 'D') {
          cmdMapDigital(cmd);
        } else if (nextChar == 'A') {
          cmdMapAnalog(cmd);
        }
      }
      break;

    case CMD_DISABLE:
      cmdDisableInput(cmd);
      break;

    case CMD_ENABLE:
      cmdEnableInput(cmd);
      break;

    case CMD_LIST:
      Serial.println("D0 D1 D2 D3 D4 D5 A0 A1");
      break;

    case CMD_RESET:
      resetConfig();
      break;

    case CMD_SAVE:
      saveConfig();
      break;

    case CMD_STATUS:
      Serial.println("STATUS OK");
      break;

    case CMD_NAME:
      cmdSetName(cmd);
      break;

    case CMD_DEADZONE:
      cmdDeadzone(cmd);
      break;

    case CMD_INVERT:
      cmdInvert(cmd);
      break;

    case CMD_SCALE:
      cmdScale(cmd);
      break;

    case CMD_ENCODER_AXIS:
      cmdEncoderAxis(cmd);
      break;

    case CMD_ENCODER_STEP:
      cmdEncoderStep(cmd);
      break;

    default:
      Serial.println("UNKNOWN COMMAND");
      break;
  }
}

//================================================
// SETUP
//================================================

void setup() {
  Serial.begin(115200);
  
  delay(100);
  Serial.println("\n\n=== BLE USB Slave Starting ===\n");

  // Configure pin modes (6 digital inputs)
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);

  // Initialize encoder
  lastEncoderA = digitalRead(ENC_A);
  analogReadResolution(12);

  // Load configuration
  setDefaultMapping();
  setDefaultAnalogConfig();
  setDefaultEncoderConfig();
  loadConfig();

  // Detect operation mode
  detectMode();
  Serial.print("Mode: ");
  if (deviceMode == MODE_BLE_ONLY) Serial.println("BLE ONLY");
  else if (deviceMode == MODE_ESPNOW_ONLY) Serial.println("ESP-NOW ONLY");
  else Serial.println("HYBRID");

  // Initialize communication
  if (deviceMode == MODE_BLE_ONLY || deviceMode == MODE_HYBRID) {
    Serial.println("Initializing BLE...");
    bleGamepad.begin();
  }

  if (deviceMode == MODE_ESPNOW_ONLY || deviceMode == MODE_HYBRID) {
    Serial.println("Initializing ESP-NOW...");
    initESPNow();
  }

  Serial.println("Setup complete!");
  Serial.println("Type LISTINPUTS for available commands\n");
}

//================================================
// MAIN LOOP
//================================================

void loop() {
  readInputs();
  applyMapping();

  // Update BLE if enabled
  if (deviceMode == MODE_BLE_ONLY || deviceMode == MODE_HYBRID) {
    updateBLE();
  }

  // Send ESP-NOW if enabled
  if (deviceMode == MODE_ESPNOW_ONLY || deviceMode == MODE_HYBRID) {
    if (millis() - lastESPNowSend > ESP_NOW_SEND_INTERVAL) {
      sendESPNOW();
      lastESPNowSend = millis();
    }
  }

  processSerial();
}
