/*
  Author:   firened
  File:     vigibot_ESP32_slave.ino
  Version:
  V1.0      10.2020
  V1.1      10.2020
  V2.0      07.2021
  V2.1      08.2021

  Description:
  Vigibot Pi to ESP32 over WLAN, based on the Arduino Uart example by Mike118.
  check the robot-maker post.

  pinout GPIO:
   G10 LED
   G09 IR LED

  set IP, hostname, OTAname
  RoverCA .64
  RoverCB .65
*/

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include <M5StickC.h>
#include "RoverC.h"

const char* ssid = "ssid";
const char* pass = "pass";
// Set your Static IP address
IPAddress local_IP(192, 168, 1, 64);
// Set your Gateway IP address
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);


// Meta Type :

typedef struct {
  union {
    struct {
      int16_t x;
      int16_t y;
    };
    int16_t coordonnees[2];
    uint8_t bytes[4];
  };
} Point;

typedef struct {
  union {
    struct {
      int8_t x;
      int8_t y;
      int8_t z;
    };
    uint8_t bytes[3];
  };
} Vitesses;


// CONFIG

#define NBPOSITIONS 2
#define FAILSAFE 250 // ms


// TTS

#define TTSBUFFERSIZE 255
uint8_t ttsBuffer[TTSBUFFERSIZE];
uint8_t ttsCurseur = 0;


// TX

#define TXFRAMESIZE (NBPOSITIONS * 4 + 17)

typedef struct {
  union {
    struct {
      uint8_t sync[4];               // 4
      Point positions[NBPOSITIONS];  // NBPOSITIONS * 4
      uint16_t val16[2];             // 2 * 2
      uint8_t choixCameras;          // 1
      Vitesses vitesses;             // 3
      uint8_t interrupteurs;         // 1
      uint8_t val8[4];               // 4
    };

    uint8_t bytes[TXFRAMESIZE];
  };
} TrameTx;


// RX

#define RXFRAMESIZE (NBPOSITIONS * 4 + 9)

typedef struct {
  union {
    struct {                       // Sizes
      uint8_t sync[4];              // 4
      Point positions[NBPOSITIONS]; // NBPOSITIONS * 4
      uint8_t choixCameras;         // 1
      Vitesses vitesses;            // 3
      uint8_t interrupteurs;        // 1
    };

    uint8_t bytes[RXFRAMESIZE];
  };
} TrameRx;


TrameTx trameTx;
TrameRx trameRx;

uint32_t lastTrameTimestamp = millis();
uint32_t lastSleepBeacon, lastLcdLoop;
uint32_t lcdOnUntil = 60000;
int16_t motor[4], motor_overflow = 127;
uint8_t under_pressure;
float BatVoltage;

WiFiServer server(7070);  //ESP server port
WiFiClient client;

void setup()
{
  Serial.begin(115200);

  // Initialize the M5StickC object
  M5.begin();
  M5.Axp.ScreenBreath(9);//5:off, 7:off, 9:mid
  M5.Axp.SetChargeVoltage(VOLTAGE_4100MV);
  //M5.Axp.SetVOff(VOLTAGE_OFF_2600MV);

  //M5.Lcd.setTextSize(1.5);
  M5.Lcd.setRotation(3);
  M5.Lcd.setTextColor(RED);
  M5.Lcd.setCursor(0, 30, 4);
  M5.Lcd.print("No WiFi");
  RoverC_Init();
  Move_stop(0);

  //pinMode(G10, OUTPUT);
  //digitalWrite(G10, HIGH);
  digitalWrite(M5_LED, HIGH);
  pinMode(G9, OUTPUT);//IR LED
  digitalWrite(G9, HIGH);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.setSleep(false);
  WiFi.config(local_IP, gateway, subnet);
  WiFi.setHostname("RocerCA");
  WiFi.begin(ssid, pass);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    //digitalWrite(G10, HIGH);
    M5.Lcd.print(": FAIL");
    delay(2000);
    ESP.restart();
  }
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(BLUE);
  M5.Lcd.setCursor(0, 30, 4);
  M5.Lcd.print("Wait Client");
  digitalWrite(M5_LED, LOW);

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("RoverCA");

  // No authentication by default
  ArduinoOTA.setPassword("rover");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
  .onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
    Move_stop(0);
    if (!lcdOnUntil) {
      M5.Axp.SetLDO2(1);
    }
  })
  .onEnd([]() {
    Serial.println("\nEnd");
  })
  .onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.printf("Update: %u%%", (progress / (total / 100)));
  })
  .onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();

  server.begin();

  Serial.println("Ready.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  // add all your init here
  lcdOnUntil = millis() + 30000;
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Move_stop(0);
    Serial.println("Connection Failed! reconnecting...");
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(RED);
    M5.Lcd.setCursor(0, 30, 4);
    M5.Lcd.print("Failed WiFi");
    while (WiFi.waitForConnectResult() != WL_CONNECTED) {
      Serial.println("Connection Failed! reconnecting...");
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, pass);
    }
  }
  ArduinoOTA.handle();
  checkClient();
  delay(0);
  // M5 Loop
  M5.update();
  delay(0);

  if (lastLcdLoop < millis() - 500) {
    BatVoltage = M5.Axp.GetBatVoltage();
    if (BatVoltage < 3.5) {
      M5.Lcd.fillScreen(RED);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(0, 30, 4);
      M5.Lcd.print("Low Bat");
    }
    M5.Lcd.fillRect(0, 0, 160, 20, BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.printf("M5: %.2f V, %.0f °C", BatVoltage, M5.Axp.GetTempInAXP192());
    delay(0);
    M5.Lcd.fillRect(0, 60, 160, 20, BLACK);
    M5.Lcd.setTextColor(GREEN);
    M5.Lcd.setCursor(0, 60, 2);
    M5.Lcd.printf("I: %d, r: %d, O: %d", motor[0], motor_overflow,
                  ((motor[0] * (127 / 2)) / motor_overflow) * 2);
    //M5.Lcd.printf("I: %d, r: %d, O: %d", motor[0], motor_overflow, map(motor[0], -motor_overflow, motor_overflow, -127, 127));
    delay(0);
    lastLcdLoop = millis();
  }

  if (M5.BtnB.isPressed()) {
    if (!lcdOnUntil) {
      M5.Axp.SetLDO2(1);
    }
    lcdOnUntil = millis() + 30000;
    Move_forward(10);
    delay(1);
  }
  if (M5.BtnB.wasReleased()) {
    Move_stop(0);
  }

  if (lcdOnUntil && (lcdOnUntil < millis())) {
    lcdOnUntil = 0;
    M5.Axp.SetLDO2(0);
  }
  delay(0);

  if (readPiSerial()) {
    // each time we receive a full trame run repeatedly:
    // use values inside trameRx to tell your robot how to move ...
    // trameRx.vitesses.x , trameRx.vitesses.y, trameRx.vitesses.z
    // trameRx.positions[i].x trameRx.positions[i].y  etc....

    //test return
    /*if (trameTx.interrupteurs & 0b00100000) {
      trameRx.vitesses.y = 10;
      }*/

    if (!lcdOnUntil) {
      M5.Axp.SetLDO2(1);
    }
    delay(0);
    lcdOnUntil = millis() + 60000;

    //beep
    if (trameTx.interrupteurs & 0b00000100) {
      if (!(trameRx.vitesses.y
            || trameRx.vitesses.x
            || trameRx.vitesses.z)) {
        trameRx.vitesses.y = 10;
      }
    }
    //motor 0 FL
    motor[0] = + trameRx.vitesses.y
               + trameRx.vitesses.x
               - trameRx.vitesses.z / 4;
    //motor 0 FR
    motor[1] = + trameRx.vitesses.y
               - trameRx.vitesses.x
               + trameRx.vitesses.z / 4;
    //motor 0 RL
    motor[2] = + trameRx.vitesses.y
               - trameRx.vitesses.x
               - trameRx.vitesses.z / 4;
    //motor 0 RR
    motor[3] = + trameRx.vitesses.y
               + trameRx.vitesses.x
               + trameRx.vitesses.z / 4;

    //constrain motor[] to 127, but keep ratios
    motor_overflow = 127;
    for (byte i = 0; i < 4; i++) {
      if (abs(motor[i]) > motor_overflow) {
        motor_overflow = abs(motor[i]);
      }
    }
    if (motor_overflow > 127) {
      for (byte i = 0; i < 4; i++) {
        //motor[i] = map(motor[i], -motor_overflow, motor_overflow, -127, 127);
        motor[i] = ((motor[i] * (127 / 2)) / motor_overflow) * 2;
      }
    }

    delay(0);
    if (!under_pressure) {
      //motor 0 FL
      Send_iic(0x00, motor[0]);
      delay(0);
      //motor 0 FR
      Send_iic(0x01, motor[1]);
      delay(0);
      //motor 0 RL
      Send_iic(0x02, motor[2]);
      delay(0);
      //motor 0 RR
      Send_iic(0x03, motor[3]);
      delay(0);

      //servo gripper
      Send_iic(0x10, 90 - (trameRx.positions[1].x / 256)); //364
      delay(0);
    }
    if(under_pressure == 10){
      Move_stop(0);
    }

    //LED
    if (trameTx.interrupteurs & 0b00000001) {
      digitalWrite(M5_LED, LOW);
    } else {
      digitalWrite(M5_LED, HIGH);
    }
    //IR LED
    if (trameTx.interrupteurs & 0b00000010) {
      digitalWrite(G9, LOW);
    } else {
      digitalWrite(G9, HIGH);
    }

    writePiSerial();
    lastTrameTimestamp = millis();
    delay(0);
    if(under_pressure<254){
      under_pressure++;
    }
  } else {
    under_pressure = 0;
  }
  if ( millis() - lastTrameTimestamp > FAILSAFE ) {
    if ((millis() - lastSleepBeacon > 10000) ) { // every 10 seconds
      writePiSerial();                        // Beacon to say that the robot is alive
      lastSleepBeacon = millis();
    }
    // Stop the robot in case the robot lost connection with the Pi
    Move_stop(0);
    delay(0);
  } else {
    // put your main code here, to run repeatedly:
    // avoid abstacle, run speed ...

  }
  //check for millis() overflow
  if (millis() < lastSleepBeacon) {
    lastSleepBeacon = millis();
  }
  if (millis() < lastLcdLoop) {
    lastLcdLoop = millis();
  }
  if (lcdOnUntil && (abs(lcdOnUntil - millis()) > 120000)) {
    lcdOnUntil = millis();
  }
}

void checkClient() {
  static byte lastClientState = 2;
  if (client.connected()) {
    if (lastClientState != 0) {
      lastClientState = 0;
      Serial.println("New Client.");
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(GREEN);
      M5.Lcd.setCursor(0, 30, 4);
      M5.Lcd.print("Connected");
    }
  } else {
    if (lastClientState == 0) { //if client is not actively connected anymore, stop and reconnect
      lastClientState = 1;
      // close the connection:
      client.stop();
      Serial.println("Client Disconnected.");
      M5.Lcd.fillScreen(BLACK);
      M5.Lcd.setTextColor(BLUE);
      M5.Lcd.setCursor(0, 30, 4);
      M5.Lcd.print("No Client");
    }
    //if (lastClientState == 2) {
    client = server.available();   // listen for incoming clients
    //}
  }
}

bool readPiSerial() {
  uint8_t current;
  static uint8_t lastType = 0;
  static uint8_t n = 0;
  static uint8_t frame[RXFRAMESIZE];
  while (client.available()) {
    delay(0);
    current = client.read();
    //Serial.write(current); //debug

    switch (n) {

      case 0:
        if (current == '$')
          n = 1;
        break;

      case 1:
        if (current != 'T' && lastType == 'T')
          writeTtsBuffer('\n');

        if (current == 'S' || current == 'T') {
          lastType = current;
          n = 2;
        } else
          n = 0;
        break;

      default:
        frame[n++] = current;

        if (n == RXFRAMESIZE) {

          if (lastType == 'T') {

            for (uint8_t i = 4; i < RXFRAMESIZE; i++) // Do not send the 4 sync data in tts
              writeTtsBuffer(frame[i]);

          } else if (lastType == 'S') {

            for (uint8_t p = 0; p < RXFRAMESIZE; p++)
              trameRx.bytes[p] = frame[p];

          }
          n = 0;
          return true;
        }
        //break;
    }
  }
  return false;
}

void writePiSerial() {
  // Header, do not modify
  trameTx.sync[0] = '$';
  trameTx.sync[1] = 'R';
  trameTx.sync[2] = ' ';
  trameTx.sync[3] = ' ';

  // modify the feedback according your need. By default we copy the trameRx content ...
  for (uint8_t i = 0; i < NBPOSITIONS; i++) {
    trameTx.positions[i].x = trameRx.positions[i].x;
    trameTx.positions[i].y = trameRx.positions[i].y;
  }
  //trameTx.val16[0] = M5.Axp.GetBatVoltage();   // Voltage (will be updated by Raspberry pi)
  trameTx.val16[0] = 3.7;   // Voltage (will be updated by Raspberry pi)
  trameTx.val16[1] = 0;   // Percent (will be updated by Raspberry pi)
  trameTx.choixCameras = trameRx.choixCameras;
  trameTx.vitesses.x = trameRx.vitesses.x;
  trameTx.vitesses.y = trameRx.vitesses.y;
  trameTx.vitesses.z = trameRx.vitesses.z;
  trameTx.interrupteurs = trameRx.interrupteurs;
  trameTx.val8[0] = 0;   // CPU load (will be updated by Raspberry pi)
  trameTx.val8[1] = 0;   // Soc temp (will be updated by Raspberry pi)
  trameTx.val8[2] = 0;   // link (will be updated by Raspberry pi)
  trameTx.val8[3] = 0;   // RSSI (will be updated by Raspberry pi)

  for ( uint8_t i = 0; i < TXFRAMESIZE; i++)
    client.write(trameTx.bytes[i]);
  delay(0);
}

void displayTtsBuffer (uint8_t * ttsBuffer, uint8_t bufferSize) {
  // you can modify this function to display text on a screen depending on your hardware...
  for ( uint8_t i = 0; i < bufferSize; i++) {
    Serial.write(ttsBuffer[i]);
  }
  Serial.println("");
}

void writeTtsBuffer( uint8_t ttsChar) {
  static uint8_t ttsCurseur = 0;
  if ( ttsCurseur < TTSBUFFERSIZE && ttsChar != '\n') {
    ttsBuffer[ttsCurseur] = ttsChar;
    ttsCurseur ++;
  }
  if ( ttsCurseur == TTSBUFFERSIZE || ttsChar == '\n') {
    displayTtsBuffer (ttsBuffer, ttsCurseur);
    ttsCurseur = 0;
  }
}
