#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BleGamepad.h>
#include <Wire.h>
#include <MPU6050_light.h>

//================================================
// PIN DEFINITIONS (6 BUTTONS + 2 TRIGGER BUTTONS + MPU6050)
//================================================

#define D0 27
#define D1 14
#define D2 19
#define D3 18
#define D4 17
#define D5 16
#define TRIGGER_RIGHT 25  // Right trigger (Z axis to max)
#define TRIGGER_LEFT 26   // Left trigger (Z axis to min)

// MPU6050 I2C pins
#define SDA_PIN 22
#define SCL_PIN 21

//================================================
// CONFIGURATION CONSTANTS
//================================================

#define AXIS_MIN -32767
#define AXIS_MAX 32767
#define MAX_BUTTONS 32
#define MAX_DIGITAL_INPUTS 8  // 6 buttons + 2 triggers
#define MAX_AXES 3
#define ESP_NOW_SEND_INTERVAL 5  // milliseconds
#define DEVICE_NAME_SIZE 32
#define MAC_ADDRESS_SIZE 6
#define DEBOUNCE_MS 20  // Debounce delay in milliseconds

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
char deviceName[DEVICE_NAME_SIZE] = "EGP-01";
BleGamepad bleGamepad(deviceName, "LEHIVXX", 100);

uint8_t masterAddress[MAC_ADDRESS_SIZE] = {0xCC, 0x8D, 0xA2, 0xEC, 0xDC, 0xAC};

bool digitalState[8];  // 6 buttons + 2 triggers
bool digitalPrevState[8];
unsigned long lastDebounceTime[8] = {0};  // Track debounce timing per input
uint32_t buttons = 0;
uint32_t lastButtons = 0;  // Track previous button state for efficient BLE updates
int16_t axis[MAX_AXES];

//================================================
// TRIGGER STATE
//================================================

bool triggerRightPressed = false;
bool triggerLeftPressed = false;
int16_t triggerAxis = 0;  // Axis 2 (Z axis) controlled by triggers

//================================================
// MPU6050 STATE
//================================================

MPU6050 mpu(Wire);
bool mpu6050_enabled = false;  // Flag to indicate MPU6050 availability
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

AnalogConfig analogCfg[3];
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
  for (int i = 0; i < 3; i++) {
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
    if (n.length() < DEVICE_NAME_SIZE) {
      n.toCharArray(deviceName, sizeof(deviceName) - 1);
      deviceName[sizeof(deviceName) - 1] = '\0';
    }
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
    Serial.println("WARNING: MPU6050 disabled. Using fallback mode.");
    mpu6050_enabled = false;
    return;  // Don't block; allow device to function without MPU
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

  mpu6050_enabled = true;
  Serial.println("MPU6050 Ready");
}

//================================================
// MPU6050 READING WITH SMOOTHING
//================================================

void readMPU6050() {
  if (!mpu6050_enabled) return;
  
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

    // Calculate average whenever buffer is full
    if (sampleCount == MPU_BUFFER_SIZE) {
      float sum = 0.0f;

      for (int i = 0; i < MPU_BUFFER_SIZE; i++) {
        sum += sampleBuffer[i];
      }

      float avgAngle = sum / MPU_BUFFER_SIZE;
      avgAngle = constrain(avgAngle, -MPU_MAX_ANGLE, MPU_MAX_ANGLE);

      // Linear steering mapping from -80 to 80 degrees
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
// APPLY AXIS CONFIGURATION (DEADZONE, INVERT, SCALE)
//================================================

int16_t applyAxisConfig(int16_t value, int axisIdx) {
  if (axisIdx < 0 || axisIdx >= 3)
    return value;

  // Apply deadzone
  if (abs(value) < analogCfg[axisIdx].deadzone)
    return 0;

  // Apply invert
  if (analogCfg[axisIdx].invert)
    value = -value;

  // Apply scaling (map from -32767..32767 to configured range)
  int16_t result = constrain(
    map(value, -32768, 32767, analogCfg[axisIdx].outMin, analogCfg[axisIdx].outMax),
    -32767, 32767
  );

  return result;
}

//================================================
// INPUT READING WITH DEBOUNCING
//================================================

void readInputs() {
  unsigned long now = millis();

  // Read digital inputs (6 buttons)
  bool newDigitalState[6] = {
    !digitalRead(D0),
    !digitalRead(D1),
    !digitalRead(D2),
    !digitalRead(D3),
    !digitalRead(D4),
    !digitalRead(D5)
  };

  // Apply debouncing
  for (int i = 0; i < 6; i++) {
    if ((now - lastDebounceTime[i]) > DEBOUNCE_MS) {
      if (newDigitalState[i] != digitalPrevState[i]) {
        digitalState[i] = newDigitalState[i];
        digitalPrevState[i] = newDigitalState[i];
        lastDebounceTime[i] = now;
      }
    }
  }

  // Read trigger inputs (GPIO 25, 26) - no debouncing for triggers (used as analog)
  triggerRightPressed = !digitalRead(TRIGGER_RIGHT);
  triggerLeftPressed = !digitalRead(TRIGGER_LEFT);

  // Process triggers for axis 2 (Z axis)
  // Right trigger (GPIO 25): Z axis goes to max
  // Left trigger (GPIO 26): Z axis goes to min
  // Both released or both pressed: Z axis goes to center
  if (triggerRightPressed && !triggerLeftPressed) {
    triggerAxis = AXIS_MAX;
  } else if (!triggerRightPressed && triggerLeftPressed) {
    triggerAxis = AXIS_MIN;
  } else {
    // Both released or both pressed -> center
    triggerAxis = 0;
  }
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

  // MPU6050 smoothed value (-80 to 80 degrees) is mapped to axis 0 (left stick X) with configuration applied
  axis[0] = applyAxisConfig(mpuAxis, 0);
  
  // Axis 1 unused (set to 0)
  axis[1] = 0;
  
  // Trigger-based axis 2 (Z axis) with configuration applied
  axis[2] = applyAxisConfig(triggerAxis, 2);
}

//================================================
// BLE COMMUNICATION (OPTIMIZED)
//================================================

void updateBLE() {
  if (!bleGamepad.isConnected()) return;

  // Only update buttons if they changed
  if (buttons != lastButtons) {
    for (int i = 0; i < 6; i++) {
      bool wasPressed = (lastButtons & (1UL << i)) != 0;
      bool isPressed = (buttons & (1UL << i)) != 0;
      
      if (wasPressed != isPressed) {
        if (isPressed) {
          bleGamepad.press(i + 1);
        } else {
          bleGamepad.release(i + 1);
        }
      }
    }
    lastButtons = buttons;
  }

  // Update axes (3 axes: X, Y, Z)
  bleGamepad.setAxes(
    axis[0], axis[1], axis[2], 0,
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
  // Copy 3 axes, rest are 0
  packet.axis[0] = axis[0];
  packet.axis[1] = axis[1];
  packet.axis[2] = axis[2];
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
  delay(100);  // Debounce delay
  
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
  
  if (name.length() == 0) {
    Serial.println("NAME EMPTY");
    return;
  }
  
  if (name.length() >= DEVICE_NAME_SIZE) {
    Serial.println("NAME TOO LONG");
    return;
  }
  
  name.toCharArray(deviceName, sizeof(deviceName) - 1);
  deviceName[sizeof(deviceName) - 1] = '\0';
  Serial.print("NAME SET TO: ");
  Serial.println(deviceName);
}

void cmdDeadzone(const String& cmd) {
  int axisNum;
  int value;

  if (sscanf(cmd.c_str(), "DEADZONE A%d %d", &axisNum, &value) != 2) {
    Serial.println("INVALID DEADZONE SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= 3) {
    Serial.println("INVALID ANALOG INDEX (A0-A2)");
    return;
  }

  if (value < 0 || value > 32767) {
    Serial.println("DEADZONE OUT OF RANGE (0-32767)");
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

  if (axisNum < 0 || axisNum >= 3) {
    Serial.println("INVALID ANALOG INDEX (A0-A2)");
    return;
  }

  analogCfg[axisNum].invert = !analogCfg[axisNum].invert;
  Serial.print("INVERT A");
  Serial.print(axisNum);
  Serial.print(" = ");
  Serial.println(analogCfg[axisNum].invert ? "ON" : "OFF");
}

void cmdScale(const String& cmd) {
  int axisNum, minv, maxv;

  if (sscanf(cmd.c_str(), "SCALE A%d %d %d", &axisNum, &minv, &maxv) != 3) {
    Serial.println("INVALID SCALE SYNTAX");
    return;
  }

  if (axisNum < 0 || axisNum >= 3) {
    Serial.println("INVALID ANALOG INDEX (A0-A2)");
    return;
  }

  if (minv < AXIS_MIN || minv > AXIS_MAX || maxv < AXIS_MIN || maxv > AXIS_MAX) {
    Serial.println("SCALE VALUES OUT OF RANGE");
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
      Serial.println("D0 D1 D2 D3 D4 D5 GPIO25(TRIGGER_R) GPIO26(TRIGGER_L) MPU6050");
      Serial.println("AXIS MAPPING:");
      Serial.println("  Axis 0 (X): MPU6050 -80 to +80 degrees");
      Serial.println("  Axis 1 (Y): Unused");
      Serial.println("  Axis 2 (Z): GPIO25=+32767, GPIO26=-32767, Released=0");
      break;

    case CMD_RESET:
      resetConfig();
      break;

    case CMD_SAVE:
      saveConfig();
      break;

    case CMD_STATUS:
      Serial.print("STATUS OK - Mode: ");
      if (deviceMode == MODE_BLE_ONLY) Serial.println("BLE ONLY");
      else if (deviceMode == MODE_ESPNOW_ONLY) Serial.println("ESP-NOW ONLY");
      else Serial.println("HYBRID");
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
  Serial.println("\n\n=== BLE USB Slave Starting (Refactored) ===\n");

  // Configure pin modes (6 digital inputs + 2 trigger inputs)
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  pinMode(TRIGGER_RIGHT, INPUT_PULLUP);
  pinMode(TRIGGER_LEFT, INPUT_PULLUP);

  // Initialize previous state
  for (int i = 0; i < 8; i++) {
    digitalPrevState[i] = false;
    lastDebounceTime[i] = 0;
  }

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
