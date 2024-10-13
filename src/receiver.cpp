#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include "ESPNowW.h"
#include <FastLED.h>

#define NUM_LEDS 12 // 0-8 Kabine, 9-11 Positionslichter
#define LED_PIN 4   // oder 2

Servo ruder;
Servo motor;
long lastPacket = 0;
uint8_t connectstate = 0;
uint8_t connectstate_prev = 0;

CRGB leds[NUM_LEDS];
uint8_t blinkstate = 0;

struct controlpacket
{
  uint8_t ruder;
  uint8_t direction; // 0 stop, 1 forward, 2 reverse
  uint8_t speed;
};

controlpacket rcdata;

void onRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if (data_len == 3)
  {
    lastPacket = millis();
    rcdata.ruder = map(data[0], 0, 255, 0, 180);

    if (data[1] == 1 || data[1] == 2)
    {
      rcdata.direction = data[1];
    }
    else
    {
      rcdata.direction = 0;
    }
    rcdata.speed = data[2];

    ruder.write(rcdata.ruder);

    // ESC-Range 25 bis 153, 90 ist stop

    Serial.print(data[2]);
    Serial.print("\t");

    if (rcdata.direction == 1)
    {
      Serial.println(map(data[2], 0, 255, 94, 153));
      motor.write(map(data[2], 0, 255, 94, 153));
    }
    else if (rcdata.direction == 2)
    {
      Serial.println(map(data[2], 0, 255, 94, 25));
      motor.write(map(data[2], 0, 255, 94, 25));
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.print("ESP Board MAC Address:  ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect();
  ESPNow.init();
  ESPNow.reg_recv_cb(onRecv);
  lastPacket = millis();

  FastLED.addLeds<WS2812B, LED_PIN, RGB>(leds, NUM_LEDS);
}

void loop()
{

  if (millis() - lastPacket > 200)
  {
    if (blinkstate == 0)
    {
      leds[NUM_LEDS - 3] = CRGB::White;
      leds[NUM_LEDS - 2] = CRGB::White;
      leds[NUM_LEDS - 1] = CRGB::White;
      blinkstate = 1;
    }
    else
    {
      leds[NUM_LEDS - 3] = CRGB::Black;
      leds[NUM_LEDS - 2] = CRGB::Black;
      leds[NUM_LEDS - 1] = CRGB::Black;
      blinkstate = 0;
    }
    for (int i = 0; i < NUM_LEDS - 3; i++)
    {
      leds[i] = CRGB::Black;
    }
  }
  else
  {
    leds[NUM_LEDS - 3] = CRGB::Red;
    leds[NUM_LEDS - 2] = CRGB::Green;
    leds[NUM_LEDS - 1] = CRGB::White;
    for (int i = 0; i < NUM_LEDS - 3; i++)
    {
      leds[i] = CRGB::Gold;
    }
  }

  FastLED.show();

  if (millis() - lastPacket > 500)
  {
    if (connectstate_prev == 1)
    {
      Serial.println("Offline");
      connectstate = 0;
      rcdata.direction = 0;
      ruder.detach();
      motor.detach();
    }
  }
  else
  {
    if (connectstate_prev == 0)
    {
      Serial.println("Online");
      connectstate = 1;
      ruder.attach(12);
      motor.attach(13);
    }
  }
  connectstate_prev = connectstate;
  delay(100);
}
