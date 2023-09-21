#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "TcpClient.hpp"
#include "kuart.hpp"

#include "imu_worker.h"
#include "convert.h"
#include <esp_task_wdt.h>

// /*********
// *********/
// // tcp values -------------------------------------------------
const char *ssid = "ASUS_90_2G";
const char *password = "*kliver*";
// const char *ssid = "GloryRio_WS";
// const char *password = "@workshop@";

const uint16_t port = 8092;
const char *host = "192.168.71.113";
TcpClient client;

void connectToWifi()
{
  WiFi.begin(ssid, password);
  Serial.println(String("Try connect to ") + ssid);

  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      break;
    }
    delay(500);
    Serial.println("...");
  }

  Serial.print("WiFi connected with IP: ");
  Serial.println(WiFi.localIP());
}

// LED pins-----------------------------------------------------------
const int led1 = 2;

// translation uart -------------------------------------------------
Kuart kuart(2); // use UART2

void setup()
{
  // init debug uart
  Serial.begin(115200);
  kuart.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(led1, OUTPUT);

  // init Wi-fi
  Serial.println("!!!!!!!!!!!WAKE UP!!!!!!!!!");
  connectToWifi();
  client.on(1, [](int len, uint8_t *data) {
    kuart.write(len, data);
    //client.write(len, data);
  });

  // command uart
  kuart.on([](int len, uint8_t *data) {
    client.write(len, data);
  });

  // get cpu frequancy
  char string[16];
  sprintf(string, "CPU Freq: %i", getCpuFrequencyMhz());
  Serial.println(string);
}

// client values -------------------
void loop()
{
  bool led_status = false;
  int status = client.clientAutoProceedNonBlock(millis(), port, host);
  if (status == CLIENT_TRY_CONNECT) {
    led_status = !led_status;
  } else if (status == CLIENT_OK) {
    led_status = true;
  }
  digitalWrite(led1, led_status ? HIGH : LOW);

  kuart.proceed();
}