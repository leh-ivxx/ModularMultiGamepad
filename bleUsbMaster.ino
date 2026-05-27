#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Keypad.h>
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
// KEYPAD CONFIGURATION (4x3 MATRIX)
//================================================
#define B1 1
#define B2 2
#define ROWS 4
#define COLS 3

#define C1 15
#define C2 16
#define C3 17
#define R1 4
#define R2 5
#define R3 6
#define R4 7

byte rowPins[ROWS] = {R1, R2, R3, R4};
byte colPins[COLS] = {C1, C2, C3};

char hexaKeys[ROWS][COLS] = {
  {'0', '1', '2'},
  {'3', '4', '5'},
  {'6', '7', '8'},
  {'9', 'A', 'B'}
};

Keypad keypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

//================================================
// SYSTEM LIMITS
//================================================

#define MAX_BUTTONS 30    // 12 master + 6 per slave * 3 = 30 total
#define MAX_AXES 8
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
  int16_t axis[8];

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
// READ MASTER BUTTONS - 4x3 KEYPAD
//================================================

void readLocalButtons()
{
  if (keypad.getKeys()) {
    for (int i = 0; i < LIST_MAX; i++) {
      if (keypad.key[i].stateChanged) {
        int buttonIndex = keypad.key[i].kchar - '0';
        
        if (buttonIndex >= 0 && buttonIndex < 12) {
          if (keypad.key[i].state == PRESSED) {
            buttons |= (1UL << buttonIndex);
          } else if (keypad.key[i].state == RELEASED) {
            buttons &= ~(1UL << buttonIndex);
          }
        }
      }
    }
  }
}

//================================================
// MERGE SLAVE INPUTS
// Slave 1 buttons: 12-17 (6 buttons)
// Slave 2 buttons: 18-23 (6 buttons)
// Slave 3 buttons: 24-29 (6 buttons)
//================================================

void mergeSlaveInputs()
{
  unsigned long now = millis();
  
  for (int i = 0; i < MAX_SLAVES; i++) {
    if (!slaves[i].active) continue;

    if (now - slaves[i].lastSeen > 1000)
      continue;

    // Offset slave buttons by 12 + (slave_id * 6)
    uint32_t slaveButtons = slaves[i].data.buttons << (12 + (i * 6));
    buttons |= slaveButtons;

    // Merge axes
    for (int a = 0; a < MAX_AXES; a++) {
      if (slaves[i].data.axis[a] != 0)
        axis[a] = slaves[i].data.axis[a];
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
    axis[6],
    axis[7]
  );
}

//================================================
// DETECT MODE - STARTUP BUTTON DETECTION
// B1 (0) pressed = BLE MODE
// B2 (1) pressed = USB + BLE DUAL MODE
// None pressed = USB MODE (default)
//================================================

void detectMode()
{
  delay(100);  // Debounce delay
  
  bool b1 = !digitalRead(B1);  // Button 0 (Row1, Col1)
  bool b2 = !digitalRead(B2);  // Button 1 (Row1, Col2)

  if (b1) {
    mode = MODE_BLE_ONLY;
    Serial.println("BLE MODE SELECTED");
  } else if (b2) {
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
pinMode(B1,INPUT_PULLUP);
  pinMode(B2,INPUT_PULLUP);
  // Initialize RGB LED
  rgb_led.begin();
  setLEDColor(255, 255, 0);  // Yellow startup
  delay(500);
  setLEDColor(0, 0, 0);

  // Keypad initialized via library
  Serial.println("Keypad initialized");

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
