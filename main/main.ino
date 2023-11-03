#include <Arduino.h>
#include <MKRWAN.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "HX711.h"
#include "DHT11.h"

// Serial-related DEFINEs

#define SERIAL_BAUDRATE 9600

// DHT-related

const int DHT_PIN = 7;
DHT11 dht(DHT_PIN);

// OneWire Temp Probe-related
const int ONEWIRE_PIN_A = 0;
const int ONEWIRE_PIN_B = 1;
OneWire oneWire_a(ONEWIRE_PIN_A);
OneWire oneWire_b(ONEWIRE_PIN_B);
DallasTemperature probe_a(&oneWire_a);
DallasTemperature probe_b(&oneWire_b);

// Scale-related
#define SCALE_MEANSF 286.53
#define SCALE_METALZERO 1258.7
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
HX711 scale;

// Battery-related
#define MAXVOLTAGE 3.3
#define MINVOLTAGE 2.5
const int BATTERY_PIN = A0;

// WAN-related 
const String appEui = "A8610A34353B6010";
const String appKey = "C1868E3C1B819DADBE3A8AEB2711949A";
LoRaModem modem;
int err_count= 0;
short con = 0;

// Buzzer-related
const int BUZZER_PIN = 10;

enum t_STATE {IDLE, CONNECTING, ERR, MEASURE, SENDING};
t_STATE State;

void init_serial() {
  Serial.begin(SERIAL_BAUDRATE);
}

void init_i2s() {

}

void init_OneWire() {
  probe_a.begin();
  probe_b.begin();
}

void init_HX711() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(SCALE_MEANSF);
}

void init_WAN() {
  modem.begin(EU868);  
}

void beep() {
  digitalWrite(BUZZER_PIN,HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN,LOW);
}

void setup() {
  init_serial();
  init_i2s();
  init_OneWire();
  init_HX711();
  init_WAN();
  beep();
  State = CONNECTING;
  delay(1000);
}

void loop() {
  short inTemp;
  short inHum;
  short tempA;
  short tempB;
  short HiveWeight;
  short batteryCharge;
  switch(State) {
    case CONNECTING:
    {
      Serial.print("Join test : ");
      Serial.println(++con);
      int ret=modem.joinOTAA(appEui, appKey);
      if (ret) {
        modem.minPollInterval(60);
        Serial.println("Connected");
        modem.dataRate(5);   // switch to SF7
        delay(100);          // because ... more stable
        err_count=0;
        State = MEASURE;
      }
      break;
    }

    case MEASURE:
    {
      // DHT Measures
      inTemp = ((float)dht.readTemperature())*10;
      inHum = ((float)dht.readHumidity())*10;
      // OneWire Measures
      probe_a.requestTemperatures();
      probe_b.requestTemperatures();
      tempA = ((float)probe_a.getTempCByIndex(0)*10);
      tempB = ((float)probe_b.getTempCByIndex(0)*10);
      // Scale Measures
      HiveWeight = (scale.get_units(10) - SCALE_METALZERO);
      // Battery Measures
      float batteryTemp = analogRead(BATTERY_PIN);
      batteryTemp = (batteryTemp / 1023.0) * 3.3;
      batteryCharge = (float)((batteryTemp - MINVOLTAGE)/(MAXVOLTAGE - MINVOLTAGE) * 100);
      State = SENDING;
      break;
    }

    case SENDING:
    {
      int err=0;
      modem.beginPacket();
      modem.write(inHum);
      modem.write(inTemp);
      modem.write(tempA);
      modem.write(tempB);
      modem.write(HiveWeight);
      modem.write(batteryCharge);  
      err = modem.endPacket();
      if ( err <= 0 ) {
        Serial.print("Error : ");
        Serial.println(err);
        // Confirmation not received - jam or coverage fault
        err_count++;
        if ( err_count > 50 ) {
          State = CONNECTING;
        }
      }
      State = IDLE;
      break;
    }
    
    case IDLE:
      delay(20000);
      State = MEASURE;
      break;

  }
}
