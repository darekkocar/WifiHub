
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

#include <printf.h>
#include <SoftReset.h>
#include <RF24.h>
#include <ESP8266.h>

#define DEBUG

#ifdef DEBUG
#define DEBUG_SERIAL(x) Serial.begin(x)
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#else
#define DEBUG_SERIAL(x)
#define DEBUG_PRINT(x) 
#define DEBUG_PRINTLN(x) 
#endif



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
  DEBUG_SERIAL(57600);
  DEBUG_PRINTLN("Setup start");
  printf_begin();  

  if (!SetupWifi())
  {
    restart();
  }
  InitializeNRF24();
  
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);


  digitalWrite(STATUS_LED_PIN, LOW);
  DEBUG_PRINTLN("Loop start");
}

void InitializeNRF24()
{
  DEBUG_PRINTLN("Initialize NRF24 start");
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
  DEBUG_PRINTLN("Initialize NRF24 end");
}

bool SetupWifi()
{
    DEBUG_PRINT("WiFi Setup begin\r\n");
    
    delay(2000);    // Delay for WiFI module start up

    if (wifi.setOprToStationSoftAP()) {
        DEBUG_PRINT("to station + softap ok\r\n");
    } else {
        DEBUG_PRINT("to station + softap err\r\n");
        return false;
    }

    if (wifi.joinAP(SSID, PASSWORD)) {
        DEBUG_PRINT("Join AP success\r\n");

        DEBUG_PRINT("IP:");
        DEBUG_PRINTLN( wifi.getLocalIP().c_str());       
    } else {
        DEBUG_PRINT("Join AP failure\r\n");
        return false;
    }
    
    if (wifi.disableMUX()) {
        DEBUG_PRINT("single ok\r\n");
    } else {
        DEBUG_PRINT("single err\r\n");
        return false;
    }
    
    DEBUG_PRINT("WiFi Setup end\r\n");
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
    //DEBUG_PRINT("create tcp ok\r\n");
  } else 
  {
    DEBUG_PRINT("create tcp err\r\n");
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
    DEBUG_PRINT("WiFi received \"");
    for (uint32_t i = 0; i < len; i++) {
      DEBUG_PRINT((char)buffer[i]);
    }
    DEBUG_PRINTLN("\"");
  }
}

void restart()
{
  DEBUG_PRINTLN("Restarting in 10s");
  delay(10000);
  asm volatile ("  jmp 0");  
}

// vim:cin:ai:sts=2 sw=2 ft=cpp
