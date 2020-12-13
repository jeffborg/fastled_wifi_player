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
#define DATA_PIN 15
#define BUFFER_SIZE 25
#define MAX_BUFFER 12 // 8 // start playing at 4 frames
#define MAX_TOTAL_POWER 2750
#define MCU_POWER 250

// #define FEEDBACK - send udp packets back

// Structure example to receive data
// Must match the sender structure
// 39 * 2 * 3 bytes each = 234bytes
typedef struct struct_message {
    uint8_t brightness; // brightness to use
    uint8_t maxPower; // max power consumption 1/20th scale (multiple by 20) for milliamps
    uint8_t ledCount; // led count
    unsigned long millis; // time message was sent
    CRGB leds[NUM_LEDS];
    unsigned long localTime; // time the pixel was clocked in - we use this in this sketch this isn't transmitted in the packet
} struct_message;

#define TIME_SYNC_UDP_SERVER 124

#ifdef FEEDBACK
enum response_types{ACK_PACKET, PLAYED_FRAME};
typedef struct struct_response {
  response_types responseType;
  unsigned long playedFrameMillis;
  unsigned long localTimeIn;
  unsigned long localTimePlayed;
} struct_response;
#endif

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
    unsigned long halfSyncDelay = (ourTime - timeSyncStarted) / 2;
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
  WiFi.begin("ESP32-80c4a24");

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
  processTimeResponse();
  Serial.print("time difference is ");
  Serial.println(timeDifference);
  Udp.begin(localUdpPort);
}

#ifdef FEEDBACK
void sendResponse(response_types type, struct_message* frame) {
  // let the sender know we just played the frame
  struct_response resp;
  resp.responseType = type;
  resp.playedFrameMillis = frame->millis;
  resp.localTimeIn = frame->localTime;
  if (type == PLAYED_FRAME) {
    resp.localTimePlayed = millis();
  } else {
    resp.localTimePlayed = 0;
  }
  Udp.beginPacket(Udp.remoteIP(), localUdpPort + 1);
  Udp.write((char *)&resp, sizeof(resp));
  Udp.endPacket();
}
#endif
 
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
    int len = Udp.read((char *)&incomingPacket, sizeof(incomingPacket) - sizeof(unsigned long));
    if (len > 0)
    {
      if (buffer.isFull()) {
        buffer.shift(); // remove an item from start
        overflow++;
      }
      incomingPacket.localTime = millis(); // store localtime at end of packet
      #ifdef FEEDBACK
      sendResponse(ACK_PACKET, &incomingPacket);
      #endif
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

        FastLED.setMaxPowerInVoltsAndMilliamps(5, (thisFrame.maxPower * 20) < MAX_TOTAL_POWER - MCU_POWER ? (thisFrame.maxPower * 20) : MAX_TOTAL_POWER - MCU_POWER);
        if (!buffer.isEmpty()) {
          nextFramePlayTime = buffer.first().millis - thisFrame.millis + millis();
        }
        // Serial.printf("First %d, this %d,  local %d\n", buffer.first().millis, thisFrame.millis, millis());
        #ifdef FEEDBACK
        sendResponse(PLAYED_FRAME, &thisFrame);
        #endif
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

