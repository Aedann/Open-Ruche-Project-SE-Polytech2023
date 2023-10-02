#include <MKRWAN.h>
#include "DHT11.h"

LoRaModem modem;

String appEui = "A8610A34353B6010";
String appKey = "C1868E3C1B819DADBE3A8AEB2711949A";

DHT11 dht(7);

bool connected;
int err_count;
short con; 
void setup() {
   Serial.begin(115200);
  while (!Serial);
  Serial.println("Welcome to MKR WAN 1300/1310 ");
   modem.begin(EU868);
   delay(1000);      // apparently the murata dislike if this tempo is removed...
   connected=false;
   err_count=0;
   con =0;
}

void loop() {
   char msg[2] = {3,4}; 
   float hum = dht.readHumidity();
   float tem = dht.readTemperature();
   Serial.println(hum);
   Serial.println(tem);
   if ( !connected ) {
    Serial.print("Join test : ");
    Serial.println(++con);
    int ret=modem.joinOTAA(appEui, appKey);
    if ( ret ) {
      connected=true;
      modem.minPollInterval(60);
      Serial.println("Connected");
      modem.dataRate(5);   // switch to SF7
      delay(100);          // because ... more stable
      err_count=0;
    }
  }

  if ( connected ) {
    int err=0;
    modem.beginPacket();  
    modem.write((short)hum);
    modem.write((short)tem);
    err = modem.endPacket();
    if ( err <= 0 ) {
      Serial.print("Error : ");
      Serial.println(err);
      // Confirmation not received - jam or coverage fault
      err_count++;
      if ( err_count > 50 ) {
        connected = false;
      }
      // wait for 2min for duty cycle with SF12 - 1.5s frame
      for ( int i = 0 ; i < 120 ; i++ ) {
        delay(1000);
      }
    } else {
      err_count = 0;
      // wait for 20s for duty cycle with SF7 - 55ms frame
      delay(20000);
      Serial.println("Message envoyé");   
    }
  }
}