#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <BleGamepad.h>

#define D0 21
#define D1 14
#define D2 19
#define D3 18

#define A0 36
#define A1 39

#define ANALOG_RES 4095

//------------------------------------------------
// MODES
//------------------------------------------------

enum Mode
{
  MODE_BLE_ONLY,
  MODE_ESPNOW_ONLY,
  MODE_HYBRID
};

Mode deviceMode;

//------------------------------------------------
// DEVICE NAME
//------------------------------------------------

char deviceName[32] = "ESP32_BASE";

BleGamepad bleGamepad(deviceName,"ESP32",100);

//------------------------------------------------
// MASTER MAC
//------------------------------------------------

uint8_t masterAddress[] = {0xCC,0x8D,0xA2,0xEC,0xDC,0xAC};

//------------------------------------------------
// INPUT STATE
//------------------------------------------------

bool digitalInputs[4];
uint16_t analogInputs[2];

//------------------------------------------------
// GAMEPAD STATE
//------------------------------------------------

uint32_t buttons=0;
int16_t axis[8];

//------------------------------------------------
// INPUT MAP
//------------------------------------------------

struct InputMap
{
  uint8_t type;
  uint8_t source;
  uint8_t target;
  bool enabled;
};

#define MAX_MAP 6
InputMap mapTable[MAX_MAP];

//------------------------------------------------
// ANALOG CONFIG
//------------------------------------------------

struct AnalogConfig
{
  int deadzone;
  bool invert;
  int outMin;
  int outMax;
};

AnalogConfig analogCfg[2];

//------------------------------------------------
// STORAGE
//------------------------------------------------

Preferences prefs;

//------------------------------------------------
// ESPNOW PACKET
//------------------------------------------------

typedef struct
{
  char name[32];
  uint32_t buttons;
  int16_t axis[8];

} GamepadPacket;

GamepadPacket packet;

//------------------------------------------------
// DEFAULT CONFIG
//------------------------------------------------

void defaultMap()
{
  mapTable[0]={0,0,0,true};
  mapTable[1]={0,1,1,true};
  mapTable[2]={0,2,2,true};
  mapTable[3]={0,3,3,true};

  mapTable[4]={1,0,0,true};
  mapTable[5]={1,1,1,true};
}

void defaultAnalogConfig()
{
  for(int i=0;i<2;i++)
  {
    analogCfg[i].deadzone=0;
    analogCfg[i].invert=false;
    analogCfg[i].outMin=-32767;
    analogCfg[i].outMax=32767;
  }
}

//------------------------------------------------
// LOAD CONFIG
//------------------------------------------------

void loadConfig()
{
  prefs.begin("cfg",true);

  if(prefs.isKey("map"))
    prefs.getBytes("map",mapTable,sizeof(mapTable));

  if(prefs.isKey("analog"))
    prefs.getBytes("analog",analogCfg,sizeof(analogCfg));

  if(prefs.isKey("name"))
  {
    String n=prefs.getString("name");
    n.toCharArray(deviceName,sizeof(deviceName));
  }

  prefs.end();
}

//------------------------------------------------
// SAVE CONFIG
//------------------------------------------------

void saveConfig()
{
  prefs.begin("cfg",false);

  prefs.putBytes("map",mapTable,sizeof(mapTable));
  prefs.putBytes("analog",analogCfg,sizeof(analogCfg));
  prefs.putString("name",deviceName);

  prefs.end();

  Serial.println("CONFIG SAVED");
}

//------------------------------------------------
// RESET CONFIG
//------------------------------------------------

void resetConfig()
{
  defaultMap();
  defaultAnalogConfig();
  strcpy(deviceName,"ESP32_MODULE");

  Serial.println("CONFIG RESET (not saved)");
}

//------------------------------------------------
// READ INPUTS
//------------------------------------------------

void readInputs()
{
  digitalInputs[0]=!digitalRead(D0);
  digitalInputs[1]=!digitalRead(D1);
  digitalInputs[2]=!digitalRead(D2);
  digitalInputs[3]=!digitalRead(D3);

  analogInputs[0]=analogRead(A0);
  analogInputs[1]=analogRead(A1);
}

//------------------------------------------------
// ANALOG PROCESSING
//------------------------------------------------

int16_t processAnalog(uint8_t index,int raw)
{
  AnalogConfig &cfg=analogCfg[index];

  int center=2048;

  if(abs(raw-center)<cfg.deadzone)
    raw=center;

  long v=map(raw,0,4095,cfg.outMin,cfg.outMax);

  if(cfg.invert)
    v=-v;

  return constrain(v,cfg.outMin,cfg.outMax);
}

//------------------------------------------------
// APPLY MAPPING
//------------------------------------------------

void applyMapping()
{
  buttons=0;
  memset(axis,0,sizeof(axis));

  for(int i=0;i<MAX_MAP;i++)
  {
    InputMap &m=mapTable[i];

    if(!m.enabled) continue;

    if(m.type==0)
    {
      if(digitalInputs[m.source])
        buttons|=(1<<m.target);
    }

    if(m.type==1)
    {
      axis[m.target]=processAnalog(m.source,analogInputs[m.source]);
    }
  }
}

//------------------------------------------------
// BLE UPDATE
//------------------------------------------------

void updateBLE()
{
  if(!bleGamepad.isConnected()) return;

  for(int i=0;i<32;i++)
  {
    if(buttons&(1<<i))
      bleGamepad.press(i+1);
    else
      bleGamepad.release(i+1);
  }

  bleGamepad.setAxes(
    axis[0],axis[1],axis[2],axis[3],
    axis[4],axis[5],axis[6],axis[7]
  );
}

//------------------------------------------------
// ESPNOW SEND
//------------------------------------------------

void sendESPNOW()
{
  strcpy(packet.name,deviceName);
  packet.buttons=buttons;

  memcpy(packet.axis,axis,sizeof(axis));

  esp_now_send(masterAddress,(uint8_t*)&packet,sizeof(packet));
}

//------------------------------------------------
// INIT ESPNOW
//------------------------------------------------

void initESPNow()
{
  WiFi.mode(WIFI_STA);
  esp_now_init();

  esp_now_peer_info_t peer={};

  memcpy(peer.peer_addr,masterAddress,6);
  peer.channel=0;
  peer.encrypt=false;

  esp_now_add_peer(&peer);
}

//------------------------------------------------
// MODE SELECT
//------------------------------------------------

void detectMode()
{
  bool b1=!digitalRead(D0);
  bool b2=!digitalRead(D1);

  if(b1 && !b2)
    deviceMode=MODE_BLE_ONLY;

  else if(!b1 && b2)
    deviceMode=MODE_HYBRID;

  else
    deviceMode=MODE_ESPNOW_ONLY;
}

//------------------------------------------------
// INPUT CONTROL (NEW)
//------------------------------------------------

void disableInput(String s)
{
  int index=s.substring(1).toInt();

  if(s[0]=='D' && index<4)
    mapTable[index].enabled=false;

  if(s[0]=='A' && index<2)
    mapTable[4+index].enabled=false;

  Serial.println("INPUT DISABLED");
}

void enableInput(String s)
{
  int index=s.substring(1).toInt();

  if(s[0]=='D' && index<4)
    mapTable[index].enabled=true;

  if(s[0]=='A' && index<2)
    mapTable[4+index].enabled=true;

  Serial.println("INPUT ENABLED");
}

//------------------------------------------------
// SERIAL MAP COMMAND
//------------------------------------------------

int axisNameToIndex(String s)
{
  s.toUpperCase();

  if(s=="X") return 0;
  if(s=="Y") return 1;
  if(s=="Z") return 2;
  if(s=="RX") return 3;
  if(s=="RY") return 4;
  if(s=="RZ") return 5;
  if(s=="SL1") return 6;
  if(s=="SL2") return 7;

  return -1;
}

void serialMapCommand(String cmd)
{
  char in[8];
  char tgt[8];

  sscanf(cmd.c_str(),"MAP %s %s",in,tgt);

  String input=String(in);
  String target=String(tgt);

  input.toUpperCase();
  target.toUpperCase();

  if(input[0]=='D')
  {
    int src=input.substring(1).toInt();
    int btn=target.substring(1).toInt();

    mapTable[src].type=0;
    mapTable[src].source=src;
    mapTable[src].target=btn;
    mapTable[src].enabled=true;

    Serial.println("DIGITAL MAP UPDATED");
  }

  if(input[0]=='A')
  {
    int src=input.substring(1).toInt();
    int axisIndex=axisNameToIndex(target);

    if(axisIndex>=0)
    {
      int mapIndex=4+src;

      mapTable[mapIndex].type=1;
      mapTable[mapIndex].source=src;
      mapTable[mapIndex].target=axisIndex;
      mapTable[mapIndex].enabled=true;

      Serial.println("ANALOG MAP UPDATED");
    }
  }
}

//------------------------------------------------
// SERIAL COMMANDS
//------------------------------------------------

void processSerial()
{
  if(!Serial.available()) return;

  String cmd=Serial.readStringUntil('\n');
  cmd.trim();

  if(cmd.startsWith("MAP"))
    serialMapCommand(cmd);

  else if(cmd.startsWith("DISABLE"))
    disableInput(cmd.substring(8));

  else if(cmd.startsWith("ENABLE"))
    enableInput(cmd.substring(7));

  else if(cmd=="LISTINPUTS")
    Serial.println("D0 D1 D2 D3 A0 A1");

  else if(cmd=="RESETCONFIG")
    resetConfig();

  else if(cmd=="SAVE")
    saveConfig();

  else if(cmd=="STATUS")
    Serial.println("STATUS OK");

  else if(cmd.startsWith("NAME"))
  {
    String name=cmd.substring(5);
    name.toCharArray(deviceName,sizeof(deviceName));
  }

  else if(cmd.startsWith("DEADZONE"))
  {
    int axis=cmd.substring(9,11)=="A0"?0:1;
    int value=cmd.substring(12).toInt();
    analogCfg[axis].deadzone=value;
  }

  else if(cmd.startsWith("INVERT"))
  {
    int axis=cmd.substring(7,9)=="A0"?0:1;
    analogCfg[axis].invert=!analogCfg[axis].invert;
  }

  else if(cmd.startsWith("SCALE"))
  {
    int axis;
    int minv,maxv;

    sscanf(cmd.c_str(),"SCALE A%d %d %d",&axis,&minv,&maxv);

    analogCfg[axis].outMin=minv;
    analogCfg[axis].outMax=maxv;
  }
}

//------------------------------------------------
// SETUP
//------------------------------------------------

void setup()
{
  Serial.begin(115200);

  pinMode(D0,INPUT_PULLUP);
  pinMode(D1,INPUT_PULLUP);
  pinMode(D2,INPUT_PULLUP);
  pinMode(D3,INPUT_PULLUP);

  analogReadResolution(12);

  defaultMap();
  defaultAnalogConfig();
  loadConfig();

  detectMode();

  if(deviceMode==MODE_BLE_ONLY || deviceMode==MODE_HYBRID)
    bleGamepad.begin();

  if(deviceMode==MODE_ESPNOW_ONLY || deviceMode==MODE_HYBRID)
    initESPNow();
}

//------------------------------------------------
// LOOP
//------------------------------------------------

unsigned long lastSend=0;

void loop()
{
  readInputs();
  applyMapping();

  if(deviceMode==MODE_BLE_ONLY || deviceMode==MODE_HYBRID)
    updateBLE();

  if(deviceMode==MODE_ESPNOW_ONLY || deviceMode==MODE_HYBRID)
  {
    if(millis()-lastSend>5)
    {
      sendESPNOW();
      lastSend=millis();
    }
  }

  processSerial();
}
