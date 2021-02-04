#include <Arduino.h>
/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <espnow.h>
#include <FastLED.h>
#include <CircularBuffer.h>

#ifndef NUM_LEDS
#define NUM_LEDS 33 * 2
#endif

#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define DATA_PIN D1

// from main sketch
// #define FRAMES_PER_SECOND 120
// this is what the main sketch is delaying frames by
// #define BUFFER_DELAY 150
// we need at least buffer delay space
#define BUFFER_SIZE 30

#define MAX_TOTAL_POWER 2750
#define MCU_POWER 250

// Structure example to receive data
// Must match the sender structure
// 39 * 2 * 3 bytes each = 234bytes
typedef struct struct_message {
    uint8_t brightness; // brightness to use
    uint8_t maxPower; // max power consumption 1/20th scale (multiple by 20) for milliamps
    uint8_t ledCount; // led count
    unsigned long millis; // time message was sent
    CRGB leds[NUM_LEDS];
} struct_message;

#define TIME_SYNC_UDP_SERVER 124

// Create a struct_message called myData
// struct_message myData;

CRGB leds[NUM_LEDS];
CircularBuffer<struct_message, BUFFER_SIZE> buffer;
struct_message incomingPacket;

// void loop() {
// 	unsigned int sample = analogRead(SAMPLE_PIN);
// 	if (sample != buffer.first()->value()) {
// 		Record* record = new Record(millis(), sample);
// 		buffer.unshift(record);
// 		Serial.println("---");
// 		delay(50);
// 	}
// 	if (buffer.isFull()) {
// 		Serial.println("Queue is full:");
// 		while (!buffer.isEmpty()) {
// 			Record* record = buffer.shift();
// 			record->print(&Serial);
// 			delete record;
// 			Serial.println();
// 		}
// 		Serial.println("START AGAIN");
// 	}
// }

uint16 counter = 0;
WiFiUDP Udp;
WiFiUDP TimeUdp;
unsigned int localUdpPort = 4210;  // local port to listen on

// Callback function that will be executed when data is received
// void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
//   memcpy(&myData, incomingData, len);
//   FastLED.setBrightness(myData.brightness);
//   counter++;
// }

// time a sync packet was sent off
unsigned long timeSyncStarted = 0;
// time difference between our clock and the remote clock
// note this is signed can be -ve
long timeDifference = 0;

// send time packet off
void sendTimePacket() {
  TimeUdp.beginPacket(IPAddress(192,168,4,1), TIME_SYNC_UDP_SERVER);
  unsigned long time = 0;
  TimeUdp.write((uint8_t *)&time, sizeof(time)); // send 8 byte packet to keep rtt about equal as much as possible
  timeSyncStarted = millis();
  TimeUdp.endPacket();
}

// for main loop process time response without delaying loop
boolean processTimeResponseNoDelay() {
  int cb = TimeUdp.parsePacket();
  if (cb != 0) {
    unsigned long ourTime = millis();
    unsigned long halfSyncDelay = ((ourTime - timeSyncStarted) / 2) + 2; // 2 ms fudge factor
    // buffer to stuff server time into
    unsigned long serverTime = 0;
    TimeUdp.read((char *)&serverTime, sizeof(serverTime));
    // write out new time difference to global variable
    timeDifference = (serverTime + halfSyncDelay) - ourTime;
    return true;
  }
  return false;
}

// process time response packet
boolean processTimeResponse(uint8_t timeout = 30) {
  if (timeSyncStarted == 0) {
    return false;
  }
  do {
    if (timeout == 0) {
      timeSyncStarted = 0; // reset start time
      return false; // timeout after 30ms
    }
    delay(1);
    timeout--;
  } while (!processTimeResponseNoDelay());
  return true;
}

// return the time off the remote end
unsigned long serverMillis() {
  return millis() + timeDifference;
}


void setupWifi() {
  WiFi.begin(WIFI_NAME);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  // sync time
  // TIME_SYNC_UDP_SERVER
  TimeUdp.begin(TIME_SYNC_UDP_SERVER);
  Serial.println("syncing time first");
  sendTimePacket();
  processTimeResponse(100);
  Serial.print("time difference is ");
  Serial.println(timeDifference);
  Udp.begin(localUdpPort);
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // setup fastled
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  pinMode(LED_BUILTIN, OUTPUT);
  set_max_power_indicator_LED(LED_BUILTIN);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, 100); // set low till get get some real data
  // set master brightness control this will be updated every cycle
  FastLED.setBrightness(10); // ignore my struct until we get data in there
  FastLED.showColor(CRGB::Black);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  setupWifi();

  Serial.printf("Now listening at IP %s, UDP port %d\n", WiFi.localIP().toString().c_str(), localUdpPort);

  // // Init ESP-NOW
  // if (esp_now_init() != 0) {
  //   Serial.println("Error initializing ESP-NOW");
  //   return;
  // }
  
  // // Once ESPNow is successfully Init, we will register for recv CB to
  // // get recv packer info
  // esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  // esp_now_register_recv_cb(OnDataRecv);
}

uint16_t framesPlayed = 0;
uint16_t starved = 0;
uint16_t overflow = 0;
uint8_t maxBufferItems = 0;
uint8_t minBufferItems = BUFFER_SIZE;

void loop() {

  // check wifi reconnect if disconnected
  EVERY_N_MILLIS(20) {
    if (WiFi.status() != WL_CONNECTED) {
      setupWifi();
    }
  }

  EVERY_N_MILLISECONDS(1000) {
    Serial.printf("Counter: %d, Frames Played: %d, Items in buffer: %d, Starved: %d, overflows: %d, maxItems: %d, minItems: %d\n", 
                    counter,    framesPlayed,      buffer.size(),       starved,      overflow,     maxBufferItems, minBufferItems);
    // Serial.print("last Frame time: ");
    // Serial.println(lastFrameTimeMillis);
    // Serial.print("last lcoal time: ");
    // Serial.println(lastLocalTimeMillis);
    // Serial.print("first buffer time: ");
    // Serial.println(buffer.first().millis);
    // Serial.print("last buffer time: ");
    // Serial.println(buffer.last().millis);
    // Serial.print("local time: ");
    // Serial.println(millis());
    counter = 0;
    framesPlayed = 0;
    starved = 0;
    overflow = 0;
    minBufferItems = BUFFER_SIZE;
    maxBufferItems = 0;
  }

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read((char *)&incomingPacket, sizeof(incomingPacket));
    if (len > 0)
    {
      if (!buffer.isFull()) {
        // check if clock hasn't gone backwards
        // if (buffer.size() > 0 && incomingPacket.millis < buffer.last().millis) {
        //   // update clock
        //   sendTimePacket();
        //   processTimeResponse();
        // }
        buffer.push(incomingPacket);
      } else {
        overflow++;
      }
      if (buffer.size() > maxBufferItems) {
        maxBufferItems = buffer.size();
      }
      counter++;
    }
  }

  if (!buffer.isEmpty()) { // at least 1 item in buffer
    if (buffer.first().millis < serverMillis()) {
      // frame scheduled for playback
      struct_message thisFrame = buffer.shift();
      memcpy(leds, thisFrame.leds, sizeof(thisFrame.leds));
      FastLED.setBrightness(thisFrame.brightness);
      FastLED.setMaxPowerInVoltsAndMilliamps(5, (thisFrame.maxPower * 20) < MAX_TOTAL_POWER - MCU_POWER ? (thisFrame.maxPower * 20) : MAX_TOTAL_POWER - MCU_POWER);
      FastLED.show();
      minBufferItems = MIN(buffer.size(), minBufferItems);
      framesPlayed++;
    }
  } else {
    starved++;
    // Serial.println("starved!");
  }
}

