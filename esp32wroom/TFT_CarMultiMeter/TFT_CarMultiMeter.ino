/*
  Example animated analogue meters using a ILI9341 TFT LCD screen

  Needs Font 2 (also Font 4 if using large scale label)

  Make sure all the display driver and pin connections are correct by
  editing the User_Setup.h file in the TFT_eSPI library folder.

  #########################################################################
  ###### DON'T FORGET TO UPDATE THE User_Setup.h FILE IN THE LIBRARY ######
  #########################################################################
*/

#define DEBUG_MODE 0

#define LED_1 16
#define LED_2 17

#include <TFT_eSPI.h> // Hardware-specific library
#include <SPI.h>

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define TFT_GREY 0x5AEB

#define LOOP_PERIOD 35 // Display updates every 35 ms
#define COM_PERIOD 20

float radperdeg = 0.0174532925;
float m_pi = 3.1415926536;

float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update

int old_analog =  -999; // Value last displayed
int old_digital = -999; // Value last displayed

int value[6] = {0, 0, 0, 0, 0, 0};
int old_value[6] = { -1, -1, -1, -1, -1, -1};
int min_value[6] = {0, 0, 0, 500, 0, 90};
int max_value[6] = {180, 4500, 100, 4000, 50, 150};
int value_per[6] = {0, 0, 0, 0, 0, 0};
int old_value_per[6] = { -1, -1, -1, -1, -1, -1};
int d = 0;

int idx_mainMeterValue = 1;  // Display value[idx] on mainMeter
int th_Y = 50;  // Threshold to yellow zone on mainMeter
int th_R = 75;  // Threshold to red zone on mainMeter

int tireSize = 195;  // For calcShiftPos
int flatRatio = 45;  // For calcShiftPos
int inchSize = 17;   // For calcShiftPos
float tireOutSize = ((float(tireSize) * (float(flatRatio) / 100.0f) * 2) + (float(inchSize) * 25.4f)) * m_pi;
int gearRatio[6] = {3615, 2047, 1518, 1156, 918, 794};
int finalRatio = 3944;
int clutchMeetCoef = 50;
float gearRatioAdjuster = 0.98;

//////////  Bluetooth config  //////////

//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Victor Tchistiak - 2019
//
//This example demostrates master mode bluetooth connection and pin 
//it creates a bridge between Serial and Classical Bluetooth (SPP)
//this is an extention of the SerialToSerialBT example by Evandro Copercini - 2018
//

#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

String MACadd = "00:10:CC:4F:36:03";
uint8_t address[6]  = {0x00, 0x10, 0xCC, 0x4F, 0x36, 0x03};
String name = "OBDII";
//String MACadd = "5C:56:A4:6B:B4:D6";
//uint8_t address[6]  = {0x5C, 0x56, 0xA4, 0x6B, 0xB4, 0xD6};
//String name = "Soundcore Life P2 Mini";
char *pin = "1234"; //<- standard pin would be provided by default
//bool connected = false;

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

#define BT_INIT_FLAG 0
#define REMOVE_BONDED_DEVICES 0   // <- Set to 0 to view all bonded devices addresses, set to 1 to remove

#define PAIR_MAX_DEVICES 20
uint8_t pairedDeviceBtAddr[PAIR_MAX_DEVICES][6];
char bda_str[18];

//////////  ELM327 config  //////////
#include "ELMduino.h"

ELM327 myELM327;

void setup() {
  // put your setup code here, to run once:

  #if LED_1
    pinMode(LED_1, OUTPUT);
    digitalWrite(LED_1, HIGH);
  #endif
  #if LED_2
    pinMode(LED_2, OUTPUT);
    digitalWrite(LED_2, HIGH);
  #endif

  //Serial.begin(115200); // For debug
  //Serial.begin(57600); // For debug
  Serial.begin(38400); // For debug
  //Serial.begin(9600); // For debug
  SerialBT.begin(38400);
  

  //////////  TFT setting  //////////
  tft.init();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  mainMeter(); // Draw analogue meter

  // Draw 6 linear meters
  byte d = 40;
  plotLinear("kph", 0, 160);
  plotLinear("rpm", 1 * d, 160);
  plotLinear("thrtle", 2 * d, 160);
  plotLinear("G/R", 3 * d, 160);
  plotLinear("ctr", 4 * d, 160);
  plotLinear("volt", 5 * d, 160);

  delay(5000);

  #if LED_2
    digitalWrite(LED_2, LOW);
  #endif

  //////////  Bluetooth setting  //////////

  //This example demonstrates reading and removing paired devices stored on the ESP32 flash memory
  //Sometimes you may find your ESP32 device could not connect to the remote device despite
  //many successful connections earlier. This is most likely a result of client replacing your paired
  //device info with new one from other device. The BT clients store connection info for paired devices,
  //but it is limited to a few devices only. When new device pairs and number of stored devices is exceeded,
  //one of the previously paired devices would be replaced with new one.
  //The only remedy is to delete this saved bound device from your device flash memory
  //and pair with the other device again.
  
  if (BT_INIT_FLAG) {
    initBluetooth();
    Serial.print("ESP32 bluetooth address: "); Serial.println(bda2str(esp_bt_dev_get_address(), bda_str, 18));
    // Get the numbers of bonded/paired devices in the BT module
    int count = esp_bt_gap_get_bond_device_num();
    if(!count) {
      Serial.println("No bonded device found.");
    } else {
      Serial.print("Bonded device count: "); Serial.println(count);
      if(PAIR_MAX_DEVICES < count) {
        count = PAIR_MAX_DEVICES; 
        Serial.print("Reset bonded device count: "); Serial.println(count);
      }
      esp_err_t tError =  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
      if(ESP_OK == tError) {
        for(int i = 0; i < count; i++) {
          Serial.print("Found bonded device # "); Serial.print(i); Serial.print(" -> ");
          Serial.println(bda2str(pairedDeviceBtAddr[i], bda_str, 18));     
          if(REMOVE_BONDED_DEVICES) {
            esp_err_t tError = esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
            if(ESP_OK == tError) {
              Serial.print("Removed bonded device # "); 
            } else {
              Serial.print("Failed to remove bonded device # ");
            }
            Serial.println(i);
          }
        }        
      }
    }
  }

  SerialBT.begin("ESP32test", true);
  Serial.println("The device started in master mode, make sure remote BT device is on!");
  // connect(address) is fast (upto 10 secs max), connect(name) is slow (upto 30 secs max) as it needs
  // to resolve name to address first, but it allows to connect to different devices with the same name.
  // Set CoreDebugLevel to Info to view devices bluetooth address and device names
  //connected = SerialBT.connect(name);
  //connected = SerialBT.connect(address);

/*if (connected) {
      Serial.println("Connected Succesfully!");
    } else {
      while(!SerialBT.connected(10000)) {
        Serial.println("Failed to connect. Make sure remote device is available and in range, then restart app.");
      }
    }*/

  while (!SerialBT.connect(address))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 1");
    // while(1);
  }

  while (!myELM327.begin(SerialBT, false, 2000))
  {
    Serial.println("Couldn't connect to OBD scanner - Phase 2");
    // while (1);
  }

  Serial.println("Connected to ELM327");
/*

  // disconnect() may take upto 10 secs max
  if (SerialBT.disconnect()) {
    Serial.println("Disconnected Succesfully!");
  }
  // this would reconnect to the name(will use address, if resolved) or address used with connect(name/address).
  SerialBT.connect();
*/

  #if LED_1
    digitalWrite(LED_1, LOW);
  #endif
  #if LED_2
    digitalWrite(LED_2, HIGH);
  #endif

  updateTime = millis(); // Next update time

}

int ctr = 0;
float vlt = 0.0;

void loop() {
  // put your main code here, to run repeatedly:

  //////////  ELM  //////////
  /*
  To test connection, type the following into the serial monitor:
    AT Z
    AT E0
    AT S0
    AT AL
    AT TP A0
  */
  ///*  
  if(Serial.available())
  {
    char c = Serial.read();

    Serial.write(c);
    SerialBT.write(c);
  }

  if(SerialBT.available())
  {
    char c = SerialBT.read();

    if(c == '>')
      Serial.println();

    Serial.write(c);
  }
  //*/

  //////////  BT  //////////
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }  

  //////////  TFT  //////////  
  if (updateTime <= millis()) {
    updateTime = millis() + LOOP_PERIOD;

    int kph = 0;
    float rpm = 0.0;
    float thr = 0.0;
    int gr = 9999;
    //float vlt = 0.0;

    if (DEBUG_MODE) {
      // Create a Sine wave for testing
      d += 4; if (d >= 360) d = 0;
      value[0] = 80 + 80 * sin((d + 0) * radperdeg);
      value[1] = 2500 + 2500 * sin((d + 60) * radperdeg);
      value[2] = 50 + 50 * sin((d + 120) * radperdeg);
      value[3] = 60 + 60 * sin((d + 180) * radperdeg);
      value[4] = 20 + 20 * sin((d + 240) * radperdeg);
      value[5] = 120 + 20 * sin((d + 300) * radperdeg);
    } else {
      while (rpm == 0.0){
        rpm = myELM327.rpm();
        delay(COM_PERIOD);      
      }
      while (myELM327.get_response() != 0) {
        kph = myELM327.kph();
        delay(COM_PERIOD);
      }
      kph = myELM327.conditionResponse(myELM327.findResponse(), 1, 1, 0);
      /*while (thr == 0.0){
        thr = myELM327.throttle();
        delay(COM_PERIOD);     
      }*/
      if (ctr == 50) {
        vlt = 0.0;
        while (vlt == 0.0){
          vlt = myELM327.batteryVoltage();
          delay(COM_PERIOD);     
        }
        ctr = 0;
      }
      ctr++;

      if (kph != 0){
        gr = (int)((tireOutSize * rpm * 60) / finalRatio / kph * gearRatioAdjuster);
        if (gr > 9999){
          gr = 9999;
        }
      }

      value[0] = (int)kph;
      value[1] = (int)rpm;
      //value[2] = (int)thr;
      //value[3] = (int)myELM327.oilTemp();
      value[3] = (int)gr;
      //delay(COM_PERIOD);
      //value[4] = (int)(myELM327.fuelLevel()*37/100);
      value[4] = (int)(50 - ctr);
      value[5] = (int)vlt*10;
    }

    //for debug value
    /*Serial.println("----");  
    Serial.println(rpm);
    Serial.println(thr);
    Serial.println(vlt);
    Serial.println(kph);*/

    //unsigned long t = millis();

    plotPointer();

    char buf[1]; dtostrf(calcShiftPos(value[0], value[1]), 1, 0, buf);
    // Serial.println(buf);
    plotNeedle(value[idx_mainMeterValue], 0, max_value[idx_mainMeterValue], min_value[idx_mainMeterValue], buf);

    //Serial.println(millis()-t); // Print time taken for meter update
  }

}

// #########################################################################
//  Initialize bluetooth connection
// #########################################################################

bool initBluetooth()
{
  if(!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if(esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if(esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
  return true;
}

char *bda2str(const uint8_t* bda, char *str, size_t size)
{
  if (bda == NULL || str == NULL || size < 18) {
    return NULL;
  }
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
          bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
  return str;
}

// #########################################################################
//  Draw the analogue meter on the screen
// #########################################################################
void mainMeter()
{
  // Meter outline
  tft.fillRect(0, 0, 239, 151, TFT_BLACK);
  tft.fillRect(5, 3, 230, 144, TFT_BLACK);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Text colour

  // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
  for (int i = -50; i < 51; i += 5) {
    // Long scale tick length
    int tl = 15;

    // Coodinates of tick to draw
    float sx = cos((i - 90) * radperdeg);
    float sy = sin((i - 90) * radperdeg);
    uint16_t x0 = sx * (100 + tl) + 120;
    uint16_t y0 = sy * (100 + tl) + 140;
    uint16_t x1 = sx * 100 + 120;
    uint16_t y1 = sy * 100 + 140;

    // Coordinates of next tick for zone fill
    float sx2 = cos((i + 5 - 90) * radperdeg);
    float sy2 = sin((i + 5 - 90) * radperdeg);
    int x2 = sx2 * (100 + tl) + 120;
    int y2 = sy2 * (100 + tl) + 140;
    int x3 = sx2 * 100 + 120;
    int y3 = sy2 * 100 + 140;

    // Green zone limits
    if (i >= -50 && i < -25) {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
    }

    // Green zone limits
    if (i >= -25 && i < 25) {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
    }

    // RED zone limits
    if (i >= 25 && i < 50) {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
    }

    // Short scale tick length
    if (i % 25 != 0) tl = 8;

    // Recalculate coords incase tick lenght changed
    x0 = sx * (100 + tl) + 120;
    y0 = sy * (100 + tl) + 140;
    x1 = sx * 100 + 120;
    y1 = sy * 100 + 140;

    // Draw tick
    tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

    // Check if labels should be drawn, with position tweaks
    if (i % 25 == 0) {
      // Calculate label positions
      x0 = sx * (100 + tl + 10) + 120;
      y0 = sy * (100 + tl + 10) + 140;
      switch (i / 25) {
        case -2: tft.drawCentreString("0", x0, y0 - 12, 2); break;
        case -1: tft.drawCentreString("25", x0, y0 - 9, 2); break;
        case 0: tft.drawCentreString("50", x0, y0 - 6, 2); break;
        case 1: tft.drawCentreString("75", x0, y0 - 9, 2); break;
        case 2: tft.drawCentreString("100", x0, y0 - 12, 2); break;
      }
    }

    // Now draw the arc of the scale
    sx = cos((i + 5 - 90) * radperdeg);
    sy = sin((i + 5 - 90) * radperdeg);
    x0 = sx * 100 + 120;
    y0 = sy * 100 + 140;
    // Draw scale arc, don't draw the last part
    if (i < 50) tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
  }

  //tft.drawString("%RH", 5 + 230 - 40, 119 - 20, 2); // Units at bottom right
  char* tmp = "8";
  tft.drawCentreString(tmp, 120, 70, 7); // Comment out to avoid font 6
  tft.drawRect(5, 3, 230, 119, TFT_BLACK); // Draw bezel line

  plotNeedle(0, 0, 0, -1, tmp); // Put meter needle at 0
}

// #########################################################################
// Update needle position
// This function is blocking while needle moves, time depends on ms_delay
// 10ms minimises needle flicker if text is drawn within needle sweep area
// Smaller values OK if text not in sweep area, zero for instant movement but
// does not look realistic... (note: 100 increments for full scale deflection)
// #########################################################################
void plotNeedle(int value, byte ms_delay, int max_value, int min_value, char *shiftPos)
{
  int value_per = 100 * (value - min_value) / (max_value - min_value);
  if (value_per < -10) value_per = -10; // Limit value to emulate needle end stops
  if (value_per > 110) value_per = 110;
    
  if (value_per >= 75) {
    if (shiftPos[0] == '0') {
      tft.setTextColor(TFT_YELLOW, TFT_RED);
    } else {
      tft.setTextColor(TFT_WHITE, TFT_RED);
    }
  } else {
    if (shiftPos[0] == '0') {
      tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    } else {
      tft.setTextColor(TFT_CYAN, TFT_BLACK);
    }
  }

  // char buf[8]; dtostrf(calcShiftPos(value[0], value[1]), 4, 0, buf);
  tft.drawCentreString(shiftPos, 120, 70, 7);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // Move the needle util new value reached
  while (!(value_per == old_analog)) {    
    // Erase old needle image
    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)

    for (int i = -50; i < 51; i += 5) {
      // Long scale tick length
      int tl = 15;
      // Coodinates of tick to draw
      float sx = cos((i - 90) * radperdeg);
      float sy = sin((i - 90) * radperdeg);
      uint16_t x0 = sx * (100 + tl) + 120;
      uint16_t y0 = sy * (100 + tl) + 140;
      uint16_t x1 = sx * 100 + 120;
      uint16_t y1 = sy * 100 + 140;

      // Coordinates of next tick for zone fill
      float sx2 = cos((i + 5 - 90) * radperdeg);
      float sy2 = sin((i + 5 - 90) * radperdeg);
      int x2 = sx2 * (100 + tl) + 120;
      int y2 = sy2 * (100 + tl) + 140;
      int x3 = sx2 * 100 + 120;
      int y3 = sy2 * 100 + 140;

      tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREY);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREY);
    }

    // Draw new ticks
    for (int i = -50; i < -50 + value_per; i += 5) {
      // Long scale tick length
      int tl = 15;
      // Coodinates of tick to draw
      float sx = cos((i - 90) * radperdeg);
      float sy = sin((i - 90) * radperdeg);
      uint16_t x0 = sx * (100 + tl) + 120;
      uint16_t y0 = sy * (100 + tl) + 140;
      uint16_t x1 = sx * 100 + 120;
      uint16_t y1 = sy * 100 + 140;

      // Coordinates of next tick for zone fill
      float sx2 = cos((i + 5 - 90) * radperdeg);
      float sy2 = sin((i + 5 - 90) * radperdeg);
      int x2 = sx2 * (100 + tl) + 120;
      int y2 = sy2 * (100 + tl) + 140;
      int x3 = sx2 * 100 + 120;
      int y3 = sy2 * 100 + 140;

      // Green zone limits
      if (value_per < th_Y) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_GREEN);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_GREEN);
      }

      // Green zone limits
      if (value_per >= th_Y && value_per < th_R) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
      }

      // RED zone limits
      if (value_per >= th_R) {
        tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_RED);
        tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_RED);
      }

      // Short scale tick length
      if (i % 25 != 0) tl = 8;

      // Recalculate coords incase tick lenght changed
      x0 = sx * (100 + tl) + 120;
      y0 = sy * (100 + tl) + 140;
      x1 = sx * 100 + 120;
      y1 = sy * 100 + 140;

      // Draw tick
      tft.drawLine(x0, y0, x1, y1, TFT_BLACK);

      /*// Check if labels should be drawn, with position tweaks
      if (i % 25 == 0) {
        // Calculate label positions
        x0 = sx * (100 + tl + 10) + 120;
        y0 = sy * (100 + tl + 10) + 140;
        switch (i / 25) {
          case -2: tft.drawCentreString("0", x0, y0 - 12, 2); break;
          case -1: tft.drawCentreString("25", x0, y0 - 9, 2); break;
          case 0: tft.drawCentreString("50", x0, y0 - 6, 2); break;
          case 1: tft.drawCentreString("75", x0, y0 - 9, 2); break;
          case 2: tft.drawCentreString("100", x0, y0 - 12, 2); break;
        }
      }*/

      // Now draw the arc of the scale
      sx = cos((i + 5 - 90) * radperdeg);
      sy = sin((i + 5 - 90) * radperdeg);
      x0 = sx * 100 + 120;
      y0 = sy * 100 + 140;
      // Draw scale arc, don't draw the last part
      if (i < 50) tft.drawLine(x0, y0, x1, y1, TFT_BLACK);
    }
    
    old_analog = value_per;

    // Wait before next update
    delay(ms_delay);
  }
}

// #########################################################################
//  Draw a linear meter on the screen
// #########################################################################
void plotLinear(char *label, int x, int y)
{
  int w = 36;
  tft.drawRect(x, y, w, 155, TFT_BLACK);
  tft.fillRect(x + 2, y + 19, w - 3, 155 - 38, TFT_GREY);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);
  tft.drawCentreString(label, x + w / 2, y + 2, 2);

  for (int i = 0; i < 110; i += 10)
  {
    tft.drawFastHLine(x + 20, y + 27 + i, 6, TFT_BLACK);
  }

  for (int i = 0; i < 110; i += 50)
  {
    tft.drawFastHLine(x + 20, y + 27 + i, 9, TFT_BLACK);
  }

  //tft.fillTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 - 5, TFT_RED);
  //tft.fillTriangle(x + 3, y + 127, x + 3 + 16, y + 127, x + 3, y + 127 + 5, TFT_RED);

  tft.drawCentreString("---", x + w / 2, y + 155 - 18, 2);
}

// #########################################################################
//  Adjust 6 linear meter pointer positions
// #########################################################################
void plotPointer(void)
{
  int dy = 187;
  int dy_new = 187;
  byte pw = 16;

  tft.setTextColor(TFT_GREEN, TFT_BLACK);

  // Move the 6 pointers one pixel towards new value
  for (int i = 0; i < 6; i++)
  {
    char buf[8]; dtostrf(value[i], 4, 0, buf);
    tft.drawRightString(buf, i * 40 + 36 - 5, 187 - 27 + 155 - 18, 2);

    int dx = 3 + 40 * i;
    value_per[i] = 100 * (value[i] - min_value[i]) / (max_value[i] - min_value[i]);
    if (value_per[i] < 0) value_per[i] = 0; // Limit value to emulate needle end stops
    if (value_per[i] > 100) value_per[i] = 100;

    while (!(value_per[i] == old_value_per[i]))
    {
      dy = 187 + 100 - old_value_per[i];
      dy_new = 187 + 100 - value_per[i];
      tft.fillTriangle(dx, dy, dx + pw, dy, dx, dy - 5, TFT_GREY);
      tft.fillTriangle(dx, dy, dx + pw, dy, dx, dy + 5, TFT_GREY);
      tft.fillTriangle(dx, dy_new, dx + pw, dy_new, dx, dy_new - 5, TFT_RED);
      tft.fillTriangle(dx, dy_new, dx + pw, dy_new, dx, dy_new + 5, TFT_RED);
      old_value_per[i] = value_per[i];
    }
  }
}

int calcShiftPos(int kph, int rpm) {
  int calcRatio = (tireOutSize * rpm * 60) / finalRatio / kph * gearRatioAdjuster;
  int shiftPos = 0;
  for (int i = 1; i < 7; i++) {
    if (calcRatio > (gearRatio[i-1] * 0.95) && calcRatio < (gearRatio[i-1] * 1.1)){
      shiftPos = i;
      break;
    }
  }
  //Serial.println(shiftPos);
  return shiftPos;
}

