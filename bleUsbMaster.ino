#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

#include "USB.h"
#include "USBHID.h"
#include "USBHIDGamepad.h"

#include <BleGamepad.h>

//================================================
// NEOPIXEL RGB LED CONFIG
//================================================

#define RGB_LED_PIN 48
#define NUM_RGB_LEDS 1
Adafruit_NeoPixel rgb_led = Adafruit_NeoPixel(NUM_RGB_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

//================================================
// DIGITAL INPUT DEFINITIONS (8 GPIO PINS)
//================================================
#define D0 1
#define D1 2
#define D2 4
#define D3 5
#define D4 6
#define D5 7
#define D6 15
#define D7 16

//================================================
// SYSTEM LIMITS
//================================================

#define MAX_BUTTONS 26    // 8 master + 6 per slave * 3 = 26 total
#define MAX_AXES 6        // 2 axes per slave * 3 = 6 total
#define MAX_SLAVES 3

//================================================
// DEVICE MODES
//================================================

enum DeviceMode
{
  MODE_USB_ONLY,
  MODE_BLE_ONLY,
  MODE_USB_BLE_DUAL
};

DeviceMode mode;

//================================================
// RGB LED EFFECT TIMING
//================================================

unsigned long lastLEDUpdate = 0;
unsigned long lastModeChangeTime = 0;
bool modeChangeActive = false;
uint8_t modeChangeCounter = 0;

//================================================
// GAMEPAD STATE
//================================================

uint32_t buttons = 0;
int16_t axis[MAX_AXES];

//================================================
// DIGITAL INPUT STATE
//================================================

bool digitalState[8];
bool digitalPrevState[8];

//================================================
// USB + BLE DEVICES
//================================================

USBHIDGamepad usbGamepad;
BleGamepad bleGamepad("EGP-01", "LEHIVXX", 31);

//================================================
// SLAVE PACKET FORMAT
//================================================

typedef struct
{
  char name[32];
  uint32_t buttons;
  int16_t axis[6];

} GamepadPacket;

//================================================
// SLAVE STORAGE
//================================================

struct SlaveDevice
{
  uint8_t mac[6];
  GamepadPacket data;
  bool active;
  unsigned long lastSeen;
};

SlaveDevice slaves[MAX_SLAVES];

//================================================
// RGB LED EFFECTS - POWER EFFICIENT
//================================================

void setLEDColor(uint8_t r, uint8_t g, uint8_t b)
{
  rgb_led.setPixelColor(0, rgb_led.Color(r, g, b));
  rgb_led.show();
}

void blinkEffect(uint8_t r, uint8_t g, uint8_t b, uint8_t count, uint16_t duration)
{
  for (uint8_t i = 0; i < count; i++) {
    setLEDColor(r, g, b);
    delay(duration / 2);
    setLEDColor(0, 0, 0);
    delay(duration / 2);
  }
}

void statusBlinkBySlaves(uint8_t numSlaves)
{
  // Blink pattern based on number of connected slaves
  // 1 slave: 1 blink red
  // 2 slaves: 2 blinks orange
  // 3 slaves: 3 blinks yellow
  
  uint8_t r = 255, g = 0, b = 0;
  
  if (numSlaves >= 2) {
    r = 255;
    g = 165;
    b = 0;  // Orange
  }
  if (numSlaves >= 3) {
    r = 255;
    g = 255;
    b = 0;  // Yellow
  }
  
  blinkEffect(r, g, b, numSlaves, 300);
}

// Breathing effect - smooth and efficient
void breathingEffect(uint8_t r, uint8_t g, uint8_t b, uint16_t duration)
{
  static unsigned long breathStart = 0;
  unsigned long elapsed = millis() - breathStart;
  
  if (elapsed > duration) {
    breathStart = millis();
    elapsed = 0;
  }
  
  // Sine wave brightness 0-255
  float progress = (float)elapsed / duration;
  uint8_t brightness = (sin(progress * 2 * PI) + 1) * 127.5;
  
  rgb_led.setPixelColor(0, rgb_led.Color(
    (r * brightness) / 255,
    (g * brightness) / 255,
    (b * brightness) / 255
  ));
  rgb_led.show();
}

//================================================
// UPDATE LED STATUS
//================================================

void updateLEDStatus()
{
  unsigned long now = millis();
  
  // Mode change animation (first 3 seconds)
  if (modeChangeActive && (now - lastModeChangeTime < 3000)) {
    if (mode == MODE_BLE_ONLY) {
      blinkEffect(0, 0, 255, 1, 500);  // Blue for BLE
    } else if (mode == MODE_USB_ONLY) {
      blinkEffect(0, 255, 0, 1, 500);  // Green for USB
    } else {
      blinkEffect(255, 0, 255, 1, 500);  // Magenta for DUAL
    }
    modeChangeCounter++;
    if (modeChangeCounter >= 3) {
      modeChangeActive = false;
    }
  }
  
  // Normal operation: breathing effect showing connected slaves
  if (!modeChangeActive) {
    uint8_t activeSlaves = 0;
    for (int i = 0; i < MAX_SLAVES; i++) {
      if (slaves[i].active && (now - slaves[i].lastSeen < 1000)) {
        activeSlaves++;
      }
    }
    
    if (mode == MODE_BLE_ONLY) {
      if (bleGamepad.isConnected()) {
        breathingEffect(0, 0, 255, 2000);  // Breathing blue for BLE connected
      } else {
        // Pulsing without breathing (off)
        if ((now / 500) % 2 == 0) {
          setLEDColor(0, 0, 100);  // Dim blue when not connected
        } else {
          setLEDColor(0, 0, 0);
        }
      }
    } else if (mode == MODE_USB_ONLY) {
      breathingEffect(0, 255, 0, 2000);  // Breathing green for USB
    } else {
      // DUAL mode - breathing magenta
      breathingEffect(255, 0, 255, 2000);
    }
    
    // Occasional slave status indication
    if (activeSlaves > 0 && (now / 5000) % 4 == 0) {
      statusBlinkBySlaves(activeSlaves);
    }
  }
}

//================================================
// FIND / REGISTER SLAVE
//================================================

int findSlave(uint8_t *mac)
{
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (memcmp(slaves[i].mac, mac, 6) == 0)
      return i;
  }

  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].active) {
      memcpy(slaves[i].mac, mac, 6);
      slaves[i].active = true;

      Serial.print("New slave registered: ");

      for (int b = 0; b < 6; b++) {
        Serial.print(mac[b], HEX);
        Serial.print(":");
      }

      Serial.println();

      return i;
    }
  }

  return -1;
}

//================================================
// ESP-NOW RECEIVE
//================================================

void onReceive(const uint8_t *mac, const uint8_t *data, int len)
{
  if (len != sizeof(GamepadPacket)) return;

  GamepadPacket packet;

  memcpy(&packet, data, sizeof(packet));

  int id = findSlave((uint8_t*)mac);

  if (id < 0) return;

  slaves[id].data = packet;
  slaves[id].lastSeen = millis();
}

//================================================
// INIT ESP-NOW
//================================================

void initESPNow()
{
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW INIT FAILED");
    return;
  }

  esp_now_register_recv_cb(onReceive);

  Serial.println("ESP-NOW READY");
}

//================================================
// READ MASTER BUTTONS - 8 DIGITAL INPUTS
// D0-D7 map to gamepad buttons 1-8
//================================================

void readLocalButtons()
{
  // Read current state of all 8 digital inputs
  digitalState[0] = !digitalRead(D0);
  digitalState[1] = !digitalRead(D1);
  digitalState[2] = !digitalRead(D2);
  digitalState[3] = !digitalRead(D3);
  digitalState[4] = !digitalRead(D4);
  digitalState[5] = !digitalRead(D5);
  digitalState[6] = !digitalRead(D6);
  digitalState[7] = !digitalRead(D7);

  // Process press and release events
  for (int i = 0; i < 8; i++) {
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
}

//================================================
// MERGE SLAVE INPUTS
// Slave 1 buttons: 8-13 (6 buttons)
// Slave 2 buttons: 14-19 (6 buttons)
// Slave 3 buttons: 20-25 (6 buttons)
// Slave axes: axes 0-1 per slave
//================================================

void mergeSlaveInputs()
{
  unsigned long now = millis();
  
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].active) continue;

    if (now - slaves[i].lastSeen > 1000)
      continue;

    // Offset slave buttons by 8 + (slave_id * 6)
    uint32_t slaveButtons = slaves[i].data.buttons << (8 + (i * 6));
    buttons |= slaveButtons;

    // Merge axes (each slave has 2 axes)
    for (int a = 0; a < 2; a++) {
      if (slaves[i].data.axis[a] != 0)
        axis[i * 2 + a] = slaves[i].data.axis[a];
    }
  }
}

//================================================
// UPDATE USB GAMEPAD
//================================================

void updateUSB()
{
  int8_t x  = constrain(axis[0] / 256, -127, 127);
  int8_t y  = constrain(axis[1] / 256, -127, 127);
  int8_t z  = constrain(axis[2] / 256, -127, 127);
  int8_t rz = constrain(axis[3] / 256, -127, 127);
  int8_t rx = constrain(axis[4] / 256, -127, 127);
  int8_t ry = constrain(axis[5] / 256, -127, 127);

  uint8_t hat = 0;

  usbGamepad.send(
    x,
    y,
    z,
    rz,
    rx,
    ry,
    hat,
    buttons
  );
}

//================================================
// UPDATE BLE GAMEPAD
//================================================

void updateBLE()
{
  if (!bleGamepad.isConnected()) return;

  for (int i = 0; i < MAX_BUTTONS; i++) {
    if (buttons & (1UL << i))
      bleGamepad.press(i + 1);
    else
      bleGamepad.release(i + 1);
  }

  bleGamepad.setAxes(
    axis[0],
    axis[1],
    axis[2],
    axis[3],
    axis[4],
    axis[5],
    0,
    0
  );
}

//================================================
// DETECT MODE - STARTUP BUTTON DETECTION
// D0 pressed = BLE MODE
// D1 pressed = USB + BLE DUAL MODE
// None pressed = USB MODE (default)
// Mode buttons must be held during startup
//================================================

void detectMode()
{
  delay(100);  // Debounce delay
  
  bool d0 = !digitalRead(D0);  // Mode button 0
  bool d1 = !digitalRead(D1);  // Mode button 1

  if (d0) {
    mode = MODE_BLE_ONLY;
    Serial.println("BLE MODE SELECTED");
  } else if (d1) {
    mode = MODE_USB_BLE_DUAL;
    Serial.println("DUAL MODE SELECTED");
  } else {
    mode = MODE_USB_ONLY;
    Serial.println("USB MODE SELECTED");
  }
  
  modeChangeActive = true;
  lastModeChangeTime = millis();
  modeChangeCounter = 0;
}

//================================================
// SETUP
//================================================

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n\nESP32 Modular Gamepad Starting...");
  
  // Initialize digital input pins
  pinMode(D0, INPUT_PULLUP);
  pinMode(D1, INPUT_PULLUP);
  pinMode(D2, INPUT_PULLUP);
  pinMode(D3, INPUT_PULLUP);
  pinMode(D4, INPUT_PULLUP);
  pinMode(D5, INPUT_PULLUP);
  pinMode(D6, INPUT_PULLUP);
  pinMode(D7, INPUT_PULLUP);
  
  // Initialize previous state
  for (int i = 0; i < 8; i++) {
    digitalPrevState[i] = false;
  }

  // Initialize RGB LED
  rgb_led.begin();
  setLEDColor(255, 255, 0);  // Yellow startup
  delay(500);
  setLEDColor(0, 0, 0);

  Serial.println("Digital inputs initialized (D0-D7)");

  // Detect operating mode before ESP-NOW init
  detectMode();

  // Initialize ESP-NOW for slave communication
  initESPNow();

  // Initialize USB or BLE based on mode
  if (mode == MODE_USB_ONLY || mode == MODE_USB_BLE_DUAL) {
    Serial.println("USB GAMEPAD ENABLED");
    USB.begin();
    usbGamepad.begin();
  }

  if (mode == MODE_BLE_ONLY || mode == MODE_USB_BLE_DUAL) {
    Serial.println("BLE GAMEPAD ENABLED");
    bleGamepad.begin();
  }

  Serial.println("Setup complete. Ready for input.");
}

//================================================
// MAIN LOOP
//================================================

void loop()
{
  buttons = 0;
  memset(axis, 0, sizeof(axis));

  readLocalButtons();
  mergeSlaveInputs();

  // Update gamepad outputs based on mode
  if (mode == MODE_USB_ONLY) {
    updateUSB();
  } else if (mode == MODE_BLE_ONLY) {
    updateBLE();
  } else {
    // DUAL MODE
    updateUSB();
    updateBLE();
  }

  // Update LED status (non-blocking)
  updateLEDStatus();

  delay(5);
}
