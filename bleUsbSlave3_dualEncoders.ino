#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BleGamepad.h>

//================================================
// PIN DEFINITIONS - 6 BUTTONS + 2 ENCODERS
//================================================

#define D0 21
#define D1 14
#define D2 19
#define D3 18
#define D4 17
#define D5 16
#define ENC_A1 39
#define ENC_B1 36
#define ENC_A2 4
#define ENC_B2 5

//================================================
// CONFIGURATION CONSTANTS
//================================================

#define AXIS_MIN -32767
#define AXIS_MAX 32767
#define MAX_MAP 6
#define MAX_AXES 8
#define MAX_BUTTONS 32
#define ESP_NOW_SEND_INTERVAL 5  // milliseconds
#define DEVICE_NAME_SIZE 32
#define MAC_ADDRESS_SIZE 6
#define SERIAL_BUFFER_SIZE 64

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
char deviceName[DEVICE_NAME_SIZE] = "GP3-Encoders";
BleGamepad bleGamepad(deviceName, "LEHIVXX", 100);

uint8_t masterAddress[MAC_ADDRESS_SIZE] = {0xCC, 0x8D, 0xA2, 0xEC, 0xDC, 0xAC};

bool digitalInputs[6];
uint32_t buttons = 0;
int16_t axis[MAX_AXES];

//================================================
// ENCODER STATE - 2 ENCODERS
//================================================

int encoder1Accum = 0;
int encoder2Accum = 0;
int lastEncoder1A = 0;
int lastEncoder2A = 0;

EncoderConfig encoderCfg1 = {0, 1365};  // Default: axis 0, step 1365
EncoderConfig encoderCfg2 = {1, 1365};  // Default: axis 1, step 1365

//================================================
// INPUT MAPPING & CONFIGURATION
//================================================

InputMap mapTable[MAX_MAP];
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
  mapTable[0] = {0, 0, 0, true};
  mapTable[1] = {0, 1, 1, true};
  mapTable[2] = {0, 2, 2, true};
  mapTable[3] = {0, 3, 3, true};
  mapTable[4] = {0, 4, 4, true};
  mapTable[5] = {0, 5, 5, true};
}

void setDefaultEncoderConfig() {
  encoderCfg1.axis = 0;
  encoderCfg1.step = 1365;
  encoderCfg2.axis = 1;
  encoderCfg2.step = 1365;
}

//================================================
// PERSISTENT STORAGE
//================================================

void loadConfig() {
  prefs.begin("cfg", true);

  if (prefs.isKey("map"))
    prefs.getBytes("map", mapTable, sizeof(mapTable));

  if (prefs.isKey("name")) {
    String n = prefs.getString("name");
    n.toCharArray(deviceName, sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
  }

  if (prefs.isKey("encoder1"))
    prefs.getBytes("encoder1", &encoderCfg1, sizeof(encoderCfg1));

  if (prefs.isKey("encoder2"))
    prefs.getBytes("encoder2", &encoderCfg2, sizeof(encoderCfg2));

  prefs.end();
}

void saveConfig() {
  prefs.begin("cfg", false);

  prefs.putBytes("map", mapTable, sizeof(mapTable));
  prefs.putString("name", deviceName);
  prefs.putBytes("encoder1", &encoderCfg1, sizeof(encoderCfg1));
  prefs.putBytes("encoder2", &encoderCfg2, sizeof(encoderCfg2));

  prefs.end();
  Serial.println("CONFIG SAVED");
}

void resetConfig() {
  setDefaultMapping();
  setDefaultEncoderConfig();
  strncpy(deviceName, "GP3-Encoders", sizeof(deviceName) - 1);
  deviceName[sizeof(deviceName) - 1] = '\0';
  Serial.println("CONFIG RESET (not saved)");
}

//================================================
// INPUT READING
//================================================

void readInputs() {
  // Read 6 digital inputs
  digitalInputs[0] = !digitalRead(D0);
  digitalInputs[1] = !digitalRead(D1);
  digitalInputs[2] = !digitalRead(D2);
  digitalInputs[3] = !digitalRead(D3);
  digitalInputs[4] = !digitalRead(D4);
  digitalInputs[5] = !digitalRead(D5);

  // Read first encoder
  int currentA1 = digitalRead(ENC_A1);
  
  if (currentA1 != lastEncoder1A) {
    if (digitalRead(ENC_B1) != currentA1) {
      encoder1Accum += encoderCfg1.step;
    } else {
      encoder1Accum -= encoderCfg1.step;
    }
    encoder1Accum = constrain(encoder1Accum, AXIS_MIN, AXIS_MAX);
    lastEncoder1A = currentA1;
  }

  // Read second encoder
  int currentA2 = digitalRead(ENC_A2);
  
  if (currentA2 != lastEncoder2A) {
    if (digitalRead(ENC_B2) != currentA2) {
      encoder2Accum += encoderCfg2.step;
    } else {
      encoder2Accum -= encoderCfg2.step;
    }
    encoder2Accum = constrain(encoder2Accum, AXIS_MIN, AXIS_MAX);
    lastEncoder2A = currentA2;
  }
}

//================================================
// INPUT MAPPING
//================================================

void applyMapping() {
  buttons = 0;
  memset(axis, 0, sizeof(axis));

  // Set encoder axes
  if (encoderCfg1.axis < MAX_AXES) {
    axis[encoderCfg1.axis] = encoder1Accum;
  }

  if (encoderCfg2.axis < MAX_AXES) {
    axis[encoderCfg2.axis] = encoder2Accum;
  }

  // Process mapped digital inputs (6 buttons)
  for (int i = 0; i < MAX_MAP; i++) {
    InputMap& m = mapTable[i];

    if (!m.enabled) continue;

    // Digital input mapping
    if (m.type == 0) {
      if (m.source < 6 && digitalInputs[m.source]) {
        if (m.target < MAX_BUTTONS) {
          buttons |= (1UL << m.target);
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

  // Update axes (2 encoder axes)
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
    deviceMode = MODE_ESPNOW_ONLY;
  } else {
    deviceMode = MODE_HYBRID;
  }
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
  if (src < 0 || src >= 6) {
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

void cmdDisableInput(const String& cmd) {
  String input = cmd.substring(8);
  input.toUpperCase();

  if (input[0] == 'D') {
    int idx = input.substring(1).toInt();
    if (idx >= 0 && idx < 6) {
      mapTable[idx].enabled = false;
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
    if (idx >= 0 && idx < 6) {
      mapTable[idx].enabled = true;
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

void cmdEncoderAxis(const String& cmd) {
  int encoderNum, axisNum;

  if (sscanf(cmd.c_str(), "ENCODERAXIS %d %d", &encoderNum, &axisNum) != 2) {
    Serial.println("INVALID ENCODERAXIS SYNTAX");
    return;
  }

  if (encoderNum < 1 || encoderNum > 2) {
    Serial.println("INVALID ENCODER (1-2)");
    return;
  }

  if (axisNum < 0 || axisNum >= MAX_AXES) {
    Serial.print("INVALID AXIS (0-");
    Serial.print(MAX_AXES - 1);
    Serial.println(")");
    return;
  }

  if (encoderNum == 1) {
    encoderCfg1.axis = axisNum;
  } else {
    encoderCfg2.axis = axisNum;
  }

  Serial.print("ENCODER ");
  Serial.print(encoderNum);
  Serial.print(" AXIS SET TO: ");
  Serial.println(axisNum);
}

void cmdEncoderStep(const String& cmd) {
  int encoderNum, step;

  if (sscanf(cmd.c_str(), "ENCODERSTEP %d %d", &encoderNum, &step) != 2) {
    Serial.println("INVALID ENCODERSTEP SYNTAX");
    return;
  }

  if (encoderNum < 1 || encoderNum > 2) {
    Serial.println("INVALID ENCODER (1-2)");
    return;
  }

  if (step <= 0) {
    Serial.println("STEP MUST BE POSITIVE");
    return;
  }

  if (encoderNum == 1) {
    encoderCfg1.step = step;
  } else {
    encoderCfg2.step = step;
  }

  Serial.print("ENCODER ");
  Serial.print(encoderNum);
  Serial.print(" STEP SET TO: ");
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
      Serial.println("D0 D1 D2 D3 D4 D5 ENC1 ENC2");
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
  Serial.println("\n\n=== BLE USB Slave 3 (Dual Encoders) Starting ===\n");

  // Configure pin modes - 6 digital + 2 encoders
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  // Initialize encoders
  lastEncoder1A = digitalRead(ENC_A1);
  lastEncoder2A = digitalRead(ENC_A2);

  // Load configuration
  setDefaultMapping();
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
  Serial.println("Type LISTINPUTS for available inputs\n");
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
