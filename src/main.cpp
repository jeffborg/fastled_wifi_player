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

#define NUM_LEDS 39 * 2
#define LED_TYPE WS2812
#define COLOR_ORDER GRB
#define DATA_PIN LED_BUILTIN
#define MILLI_AMPS 200 // IMPORTANT: set the max milli-Amps of your power supply (4A = 4000mA)
#define FRAMES_PER_SECOND 120
#define BUFFER_SIZE 25
#define MAX_BUFFER 12 // 8 // start playing at 4 frames

// Structure example to receive data
// Must match the sender structure
// 39 * 2 * 3 bytes each = 234bytes
typedef struct struct_message {
    uint8_t brightness; // brightness to use
    uint8_t ledCount; // led count
    unsigned long millis; // time message was sent
    CRGB leds[NUM_LEDS];
} struct_message;

enum response_types{ACK_PACKET, PLAYED_FRAME};
typedef struct struct_response {
  response_types responseType;
  unsigned long playedFrameMillis;
} struct_response;

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
unsigned int localUdpPort = 4210;  // local port to listen on

// Callback function that will be executed when data is received
// void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
//   memcpy(&myData, incomingData, len);
//   FastLED.setBrightness(myData.brightness);
//   counter++;
// }

void setupWifi() {
  WiFi.begin("ESP32-80c4a24");

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" connected");
  Udp.begin(localUdpPort);
}

void sendResponse(response_types type, u_long frameTime) {
  // let the sender know we just played the frame
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  struct_response resp;
  resp.responseType = type;
  resp.playedFrameMillis = frameTime;
  Udp.write((char *)&resp, sizeof(resp));
  Udp.endPacket();
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  
  // setup fastled
  FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  FastLED.setMaxPowerInVoltsAndMilliamps(5, MILLI_AMPS);
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

bool playing = false; // is there enough buffer
unsigned long nextFramePlayTime = 0;
uint16_t framesPlayed = 0;
uint16_t starved = 0;
uint16_t overflow = 0;
uint8_t maxBufferItems = 0;
uint8_t minBufferItems = BUFFER_SIZE;

void loop() {

  // check wifi
  EVERY_N_MILLIS(20) {
    if (WiFi.status() != WL_CONNECTED) {
      setupWifi();
    }
  }

  EVERY_N_MILLISECONDS(1000) {
    Serial.print("Counter: ");
    Serial.println(counter);
    Serial.print("Frames Played: ");
    Serial.println(framesPlayed);
    Serial.print("Itmes in buffer: ");
    Serial.println(buffer.size());
    Serial.print("Starved: ");
    Serial.println(starved);
    Serial.print("Overflow: ");
    Serial.println(overflow);
    Serial.print("maxBufferItems: ");
    Serial.println(maxBufferItems);
    Serial.print("minBufferItems: ");
    Serial.println(minBufferItems);

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
    // minBufferItems = BUFFER_SIZE;
    // maxBufferItems = 0;
  }
  if (playing)  {
    minBufferItems = MIN(buffer.size(), minBufferItems);
    if (buffer.size() > maxBufferItems) {
      maxBufferItems = buffer.size();
    }
  }

  int packetSize = Udp.parsePacket();
  if (packetSize)
  {
    // receive incoming UDP packets
    // Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read((char *)&incomingPacket, sizeof(incomingPacket));
    if (len > 0)
    {
      if (buffer.isFull()) {
        buffer.shift(); // remove an item from start
        overflow++;
      }
      sendResponse(ACK_PACKET, incomingPacket.millis);
      buffer.push(incomingPacket);
      // enable playback when buffer is 1/2 full
      if (!playing && (buffer.size() >= MAX_BUFFER)) {
        Serial.println("staring playback");
        nextFramePlayTime = 0;
        playing = true;
      }
      counter++;
    }
  }
  if (playing) {
    if (!buffer.isEmpty()) { // at last 2 items in buffer
      // long frameDelay = buffer.first().millis - lastFrameTimeMillis;
      // if (frameDelay < 0) {
      //   Serial.println("frame delay was negative");
      //   Serial.println(frameDelay);
      //   frameDelay = 0;
      // }
      // EVERY_N_MILLISECONDS(200) {
      //   Serial.printf("%d > %d\n", nextFramePlayTime, millis());
      // }
      if (nextFramePlayTime <= millis()) // (lastLocalTimeMillis + frameDelay) > (millis() + frameDelay)) // if beyond current time play the frame
      {
        struct_message thisFrame = buffer.shift();
        memcpy(leds, thisFrame.leds, sizeof(thisFrame.leds));
        FastLED.setBrightness(thisFrame.brightness);
        if (!buffer.isEmpty()) {
          nextFramePlayTime = buffer.first().millis - thisFrame.millis + millis();
        }
        // Serial.printf("First %d, this %d,  local %d\n", buffer.first().millis, thisFrame.millis, millis());
        sendResponse(PLAYED_FRAME, thisFrame.millis);
        FastLED.show();
        framesPlayed++;
      }
    } else {
      starved++;
      playing = false; // wait to fill up again
      Serial.println("starved!");
    }
  }
}

