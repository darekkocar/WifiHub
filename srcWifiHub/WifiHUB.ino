
  /*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example RF Radio Ping Pair
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two different nodes,
 * connect the role_pin to ground on one.  The ping node sends the current time to the pong node,
 * which responds by sending the value back.  The ping node can then see how long the whole cycle
 * took.
 */

#include <SoftReset.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

#include <stdio.h>
#include "ESP8266.h"

#define SSID        "potocni22"//"smartphone"
#define PASSWORD    "ca229683"//"letmeinplease"
#define HOST_NAME   "184.106.153.149"
#define HOST_PORT   80
#define APIKEY      "1JYTWOV0HCXCOG08"

SoftwareSerial mySerial(8, 7); /* RX:D3, TX:D2 */
ESP8266 wifi(mySerial);

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10
// Pro mini SPI: 10 (SS), 11 (MOSI), 12 (MISO), 13 (SCK).
RF24 radio(9,10);

#define STATUS_LED_PIN      4
//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  The hardware itself specifies
// which node it is.
//
// This is done through the role_pin
//



void setup(void)
{
  Serial.begin(57600);
  Serial.println("Setup start");
  printf_begin();  

  if (!SetupWifi())
  {
    restart();
  }
  InitializeNRF24();
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);


  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.println("Loop start");
}

void InitializeNRF24()
{
  Serial.println("Initialize NRF24 start");
  radio.begin();
  radio.setPayloadSize(12);
  radio.setPALevel( RF24_PA_MAX ) ;     // Max power 
  radio.setDataRate( RF24_250KBPS ) ;   // Min speed (for better range I presume)
  radio.setCRCLength( RF24_CRC_16 ) ;   // 8 bits CRC
  radio.setRetries(15,15);               // increase the delay between retries & # of retries 
  radio.openWritingPipe(pipes[1]);
  radio.openReadingPipe(1,pipes[0]);
  radio.printDetails();
  radio.startListening();
  Serial.println("Initialize NRF24 end");
}

bool SetupWifi()
{
    Serial.print("WiFi Setup begin\r\n");
    
    delay(2000);    // Delay for WiFI module start up

    if (wifi.setOprToStationSoftAP()) {
        Serial.print("to station + softap ok\r\n");
    } else {
        Serial.print("to station + softap err\r\n");
        return false;
    }

    if (wifi.joinAP(SSID, PASSWORD)) {
        Serial.print("Join AP success\r\n");

        Serial.print("IP:");
        Serial.println( wifi.getLocalIP().c_str());       
    } else {
        Serial.print("Join AP failure\r\n");
        return false;
    }
    
    if (wifi.disableMUX()) {
        Serial.print("single ok\r\n");
    } else {
        Serial.print("single err\r\n");
        return false;
    }
    
    Serial.print("WiFi Setup end\r\n");
    return true;
}

void loop(void)
{

    // if there is data ready
    if ( radio.available() )
    {
      // Dump the payloads until we've gotten everything
      unsigned long message[2];
      while(radio.available() )
      {
        // Fetch the payload, and see if this was the last one.
        radio.read( message, 3 * sizeof(unsigned long) );

        // Spew it
        printf("Got payload [\"%lu\", \"%lu\", \"%lu\"]\r\n", message[0], message[1], message[2]);
      }


      String WiFiMessage = "field1=";
      WiFiMessage += message[0] / 1000.0; // Vcc
      WiFiMessage += "&field2=";
      WiFiMessage += message[1];  // Moisture
      WiFiMessage += "&field3="; 
      WiFiMessage += message[2] / 10.0;   // Temperature 
      updateThingSpeak(WiFiMessage);

      // Now, resume listening so we catch the next packets.
      radio.startListening();
    }
}

void updateThingSpeak(String data)
{
  printf("Sending message to thingspeak: %s\r\n", data.c_str());
  uint8_t buffer[128] = {0};

  if (wifi.createTCP(HOST_NAME, HOST_PORT)) 
  {
    //Serial.print("create tcp ok\r\n");
  } else 
  {
    Serial.print("create tcp err\r\n");
    restart();
  }


  String getStr = "GET /update?api_key=";
  getStr += APIKEY;
  getStr +="&";
  getStr += data;
  getStr += "\r\n\r\n";

  wifi.send((const uint8_t*)getStr.c_str(), getStr.length());


  uint32_t len = wifi.recv(buffer, sizeof(buffer), 10000);
  if (len > 0) {
    Serial.print("WiFi received \"");
    for (uint32_t i = 0; i < len; i++) {
      Serial.print((char)buffer[i]);
    }
    Serial.println("\"");
  }
}

void restart()
{
  Serial.println("Restarting in 10s");
  delay(10000);
  asm volatile ("  jmp 0");  
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
