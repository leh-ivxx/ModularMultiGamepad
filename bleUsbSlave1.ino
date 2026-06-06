#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BleGamepad.h>
#include <Wire.h>
#include <MPU6050_light.h>

//================================================
// PIN DEFINITIONS (6 BUTTONS + 1 ANALOG AXIS + MPU6050)
//================================================

#define D0 21
#define D1 14
#define D2 19
#define D3 18
#define D4 17
#define D5 16
#define A0 39
// A1 removed - replaced with MPU6050

// MPU6050 I2C pins
#define SDA_PIN 22
#define SCL_PIN 21

//================================================
// CONFIGURATION CONSTANTS
//================================================

#define ANALOG_RES 4095
#define ANALOG_CENTER 2048
#define AXIS_MIN -32767
#define AXIS_MAX 32767
#define MAX_BUTTONS 32
#define MAX_DIGITAL_INPUTS 6
#define MAX_AXES 2
#define ESP_NOW_SEND_INTERVAL 5  // milliseconds
#define DEVICE_NAME_SIZE 32
#define MAC_ADDRESS_SIZE 6

//================================================
// MPU6050 SMOOTHING CONFIGURATION
//================================================

#define MPU_SAMPLE_INTERVAL_US 2000  // 2ms
#define MPU_BUFFER_SIZE 10
#define MPU_SMOOTHING_ALPHA 0.1f
#define MPU_MAX_ANGLE 80.0f
#define MPU_STEERING_GAIN 1.0f
#define MPU_DEADZONE 0

//================================================
// DEVICE MODES
//================================================

enum Mode {
  MODE_BLE_ONLY,
  MODE_ESPNOW_ONLY,
  MODE_HYBRID
};

enum CommandType {
  CMD_LIST,
  CMD_RESET,
  CMD_SAVE,
  CMD_STATUS,
  CMD_NAME,
  CMD_DEADZONE,
  CMD_INVERT,
  CMD_SCALE,
  CMD_UNKNOWN
};

//================================================
// CONFIGURATION STRUCTURES
//================================================

struct AnalogConfig {
  int deadzone;
  bool invert;
  int outMin;
  int outMax;
};

typedef struct {
  char name[DEVICE_NAME_SIZE];
  uint32_t buttons;
  int16_t axis[6];
} GamepadPacket;

//================================================
// GLOBAL STATE
//================================================

Mode deviceMode;
char deviceName[DEVICE_NAME_SIZE] = "GP3-RS1";
BleGamepad bleGamepad(deviceName, "LEHIVXX", 100);

uint8_t masterAddress[MAC_ADDRESS_SIZE] = {0xCC, 0x8D, 0xA2, 0xEC, 0xDC, 0xAC};

bool digitalState[6];
bool digitalPrevState[6];
uint16_t analogInputs[2];
uint32_t buttons = 0;
int16_t axis[MAX_AXES];

//================================================
// MPU6050 STATE
//================================================

MPU6050 mpu(Wire);
float angleOffset = 0.0f;
float smoothedAngle = 0.0f;
float sampleBuffer[MPU_BUFFER_SIZE];
uint8_t sampleIndex = 0;
uint8_t sampleCount = 0;
unsigned long lastMPUSampleTime = 0;
int16_t mpuAxis = 0;

//================================================
// CONFIGURATION
//================================================

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

void setDefaultAnalogConfig() {
  for (int i = 0; i < 2; i++) {
    analogCfg[i].deadzone = 0;
    analogCfg[i].invert = false;
    analogCfg[i].outMin = AXIS_MIN;
    analogCfg[i].outMax = AXIS_MAX;
  }
}

//================================================
// PERSISTENT STORAGE
//================================================

void loadConfig() {
  prefs.begin("cfg", true);

  if (prefs.isKey("analog"))
    prefs.getBytes("analog", analogCfg, sizeof(analogCfg));

  if (prefs.isKey("name")) {
    String n = prefs.getString("name");
    n.toCharArray(deviceName, sizeof(deviceName) - 1);
    deviceName[sizeof(deviceName) - 1] = '\0';
  }

  prefs.end();
}

void saveConfig() {
  prefs.begin("cfg", false);

  prefs.putBytes("analog", analogCfg, sizeof(analogCfg));
  prefs.putString("name", deviceName);

  prefs.end();
  Serial.println("CONFIG SAVED");
}

void resetConfig() {
  setDefaultAnalogConfig();
  strncpy(deviceName, "ESP32_SLAVE", sizeof(deviceName) - 1);
  deviceName[sizeof(deviceName) - 1] = '\0';
  Serial.println("CONFIG RESET (not saved)");
}

//================================================
// MPU6050 INITIALIZATION
//================================================

void initMPU6050() {
  Wire.begin(SDA_PIN, SCL_PIN);
  
  Serial.println("Initializing MPU6050...");
  byte status = mpu.begin();

  if (status != 0) {
    Serial.print("MPU6050 Error: ");
    Serial.println(status);
    while (1) {
      delay(100);
    }
  }

  delay(1000);
  Serial.println("Calibrating MPU6050...");
  mpu.calcOffsets(true, true);

  // Initialize smoothing buffers
  for (int i = 0; i < MPU_BUFFER_SIZE; i++) {
    sampleBuffer[i] = 0.0f;
  }

  angleOffset = 0.0f;
  smoothedAngle = 0.0f;
  sampleIndex = 0;
  sampleCount = 0;
  lastMPUSampleTime = 0;
  mpuAxis = 0;

  Serial.println("MPU6050 Ready");
}

//================================================
// MPU6050 READING WITH SMOOTHING
//================================================

void readMPU6050() {
  // Update MPU continuously
  mpu.update();

  unsigned long now = micros();

  // Sample at specified interval
  if ((now - lastMPUSampleTime) >= MPU_SAMPLE_INTERVAL_US) {
    lastMPUSampleTime = now;

    // Get relative angle
    float rawAngle = mpu.getAngleX() - angleOffset;

    // Exponential smoothing
    smoothedAngle = (MPU_SMOOTHING_ALPHA * rawAngle) + ((1.0f - MPU_SMOOTHING_ALPHA) * smoothedAngle);
    smoothedAngle = constrain(smoothedAngle, -MPU_MAX_ANGLE, MPU_MAX_ANGLE);

    // Store in circular buffer
    sampleBuffer[sampleIndex] = smoothedAngle;
    sampleIndex++;

    if (sampleIndex >= MPU_BUFFER_SIZE)
      sampleIndex = 0;

    if (sampleCount < MPU_BUFFER_SIZE)
      sampleCount++;

    // Calculate average every 10 samples
    if (sampleCount == MPU_BUFFER_SIZE && sampleIndex == 0) {
      float sum = 0.0f;

      for (int i = 0; i < MPU_BUFFER_SIZE; i++) {
        sum += sampleBuffer[i];
      }

      float avgAngle = sum / MPU_BUFFER_SIZE;
      avgAngle = constrain(avgAngle, -MPU_MAX_ANGLE, MPU_MAX_ANGLE);

      // Linear steering mapping
      float normalized = avgAngle / MPU_MAX_ANGLE;
      float scaled = normalized * MPU_STEERING_GAIN;
      scaled = constrain(scaled, -1.0f, 1.0f);

      mpuAxis = (int16_t)(scaled * AXIS_MAX);

      if (abs(mpuAxis) < MPU_DEADZONE)
        mpuAxis = 0;
    }
  }
}

//================================================
// INPUT READING
//================================================

void readInputs() {
  // Read digital inputs (6 buttons) - detect press and release
  digitalState[0] = !digitalRead(D0);
  digitalState[1] = !digitalRead(D1);
  digitalState[2] = !digitalRead(D2);
  digitalState[3] = !digitalRead(D3);
  digitalState[4] = !digitalRead(D4);
  digitalState[5] = !digitalRead(D5);

  // Process button press/release
  for (int i = 0; i < 6; i++) {
    if (digitalState[i] && !digitalPrevState[i]) {
      // Button pressed
      buttons |= (1UL << i);
    } else if (!digitalState[i] && digitalPrevState[i]) {
      // Button released
      buttons &= ~(1UL << i);
    } else if (digitalState[i]) {
      // Button held
      buttons |= (1UL << i);
    }
    
    digitalPrevState[i] = digitalState[i];
  }

  // Read analog input A0
  analogInputs[0] = analogRead(A0);
  
  // Read MPU6050 smoothed value for axis 1
  // No need to read analogInputs[1] - it's now populated by readMPU6050()
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

  // Digital inputs D0-D5 are directly mapped to buttons 0-5
  for (int i = 0; i < 6; i++) {
    if (digitalState[i]) {
      buttons |= (1UL << i);
    }
  }

  // Analog input A0 is mapped to axis 0
  axis[0] = processAnalog(0, analogInputs[0]);
  
  // MPU6050 smoothed value is mapped to axis 1
  axis[1] = mpuAxis;
}

//================================================
// BLE COMMUNICATION
//================================================

void updateBLE() {
  if (!bleGamepad.isConnected()) return;

  // Update buttons (6 buttons)
  for (int i = 0; i < 6; i++) {
    if (buttons & (1UL << i)) {
      bleGamepad.press(i + 1);
    } else {
      bleGamepad.release(i + 1);
    }
  }

  // Update axes (2 axes)
  bleGamepad.setAxes(
    axis[0], axis[1], 0, 0,
    0, 0, 0, 0
  );
}

//================================================
// ESP-NOW COMMUNICATION
//================================================

void sendESPNOW() {
  strncpy(packet.name, deviceName, sizeof(packet.name) - 1);
  packet.name[sizeof(packet.name) - 1] = '\0';
  packet.buttons = buttons;
  // Copy 2 axes, rest are 0
  packet.axis[0] = axis[0];
  packet.axis[1] = axis[1];
  packet.axis[2] = 0;
  packet.axis[3] = 0;
  packet.axis[4] = 0;
  packet.axis[5] = 0;

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
// D0 pressed = BLE MODE
// D1 pressed = ESP-NOW MODE
// Otherwise = HYBRID MODE
//================================================

void detectMode() {
  bool d0 = !digitalRead(D0);
  bool d1 = !digitalRead(D1);

  if (d0 && !d1) {
    deviceMode = MODE_BLE_ONLY;
  } else if (!d0 && d1) {
    deviceMode = MODE_ESPNOW_ONLY;
  } else {
    deviceMode = MODE_HYBRID;
  }
}

//================================================
// SERIAL COMMAND PARSING
//================================================

CommandType parseCommandType(const String& cmd) {
  if (cmd == "LISTINPUTS") return CMD_LIST;
  if (cmd == "RESETCONFIG") return CMD_RESET;
  if (cmd == "SAVE") return CMD_SAVE;
  if (cmd == "STATUS") return CMD_STATUS;
  if (cmd.startsWith("NAME")) return CMD_NAME;
  if (cmd.startsWith("DEADZONE")) return CMD_DEADZONE;
  if (cmd.startsWith("INVERT")) return CMD_INVERT;
  if (cmd.startsWith("SCALE")) return CMD_SCALE;
  return CMD_UNKNOWN;
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

void processSerial() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0) return;

  switch (parseCommandType(cmd)) {
    case CMD_LIST:
      Serial.println("D0 D1 D2 D3 D4 D5 A0 MPU6050");
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

  // Initialize previous state
  for (int i = 0; i < 6; i++) {
    digitalPrevState[i] = false;
  }

  analogReadResolution(12);

  // Load configuration
  setDefaultAnalogConfig();
  loadConfig();

  // Initialize MPU6050
  initMPU6050();

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
  // Read MPU6050 with smoothing
  readMPU6050();

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
  
  delay(5);
}
