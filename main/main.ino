#include <Arduino.h>
#include <MKRWAN.h>
#include <DFRobot_B_LUX_V30B.h>
#include <ArduinoLowPower.h>
#include "DHT22.h"

// Serial-related DEFINEs
#define USE_SERIAL false
#define SERIAL_BAUDRATE 9600

// DHT-related
const int DHT_IN_PIN = 3;
const int DHT_OUT_PIN = 1;
DHT22 dht_in(DHT_IN_PIN);
DHT22 dht_out(DHT_OUT_PIN);

short unsigned binVec[10];

//Luxmeter-related
DFRobot_B_LUX_V30B myLux(0);

//Loadswitch-related
const int LOADSWITCH_DELAY_PIN = 7;
const int LOADSWITCH_DONE_PIN = 6;

// Power-related
#define WAKEUP_DELAY_MS 6000
int sleep_time_ms = 1200000;

// WAN-related 
const String appEui = "A8610A34353B6010";
const String appKey = "C1868E3C1B819DADBE3A8AEB2711949A";
LoRaModem modem;
int err_count= 0;
short con = 0;

// Buzzer-related
const int BUZZER_PIN = 10;

enum t_STATE {CONNECTING, WAKING, SENDING, SLEEP};
t_STATE State;

void init_serial() {
  Serial.begin(SERIAL_BAUDRATE);
}

void init_Lux() {
  myLux.begin();
}

void init_WAN() {
  modem.begin(EU868);  
}

void wake_TPL() {
  digitalWrite(LOADSWITCH_DONE_PIN, LOW);
  delay(200);
  digitalWrite(LOADSWITCH_DELAY_PIN, HIGH);
  delay(800);
  digitalWrite(LOADSWITCH_DELAY_PIN, LOW);
}

void done_TPL() {
  digitalWrite(LOADSWITCH_DELAY_PIN, LOW);
  delay(200);
  digitalWrite(LOADSWITCH_DONE_PIN, HIGH);
  delay(800);
  digitalWrite(LOADSWITCH_DONE_PIN, LOW);
}

void beep() {
  digitalWrite(BUZZER_PIN,HIGH);
  delay(1000);
  digitalWrite(BUZZER_PIN,LOW);
}

void setup() {
  
  digitalWrite(LOADSWITCH_DELAY_PIN, LOW); // We need to ensure that the TPL doesn't get mixed signals at boot
  digitalWrite(LOADSWITCH_DONE_PIN, LOW);
  delay(1000);
  wake_TPL();
  delay(1000);

  // Serial.println("INIT !");

  // if(USE_SERIAL) init_serial();
  init_WAN();
  init_OneWire();
  init_HX711();
  // init_ADC();
  init_Lux();
  beep();

  State = CONNECTING;
  delay(1000);
}

void loop() {
  switch(State) {
    case CONNECTING:
    {
      // Serial.print("Join test : ");
      // Serial.println(++con);
      int ret=modem.joinOTAA(appEui, appKey);
      if (ret) {
        modem.minPollInterval(60);
        // Serial.println("Connected");
        modem.dataRate(5);   // switch to SF7
        delay(100);          // because ... more stable
        err_count=0;
        State = WAKING;
      }
      break;
    }

    case WAKING:
    {
      myFFT(binVec);
      wake_TPL();
      // Serial.println("WAKING UP !");
      
      delay(2000);
      State = SENDING;

      break;
    }

    case SENDING:
    {
      // DHT Measures
      short inTemp, inHum, outTemp, outHum;
      inTemp = ((float)dht_in.getTemperature())*10;
      inHum = ((float)dht_in.getHumidity())*10;
      outTemp = ((float)dht_out.getTemperature())*10;
      outHum = ((float)dht_out.getHumidity())*10;
      // OneWire Measures
      short tempA, tempB;
      oneWire_read_temperatures(&tempA, &tempB);
      tempA-=8;
      // Scale Measures
      short HiveWeight = getWeight();
      // Battery Measures
      short batteryVoltage = 0;
      // batteryVoltage = read_battery_voltage();
      // Audio Measures
      // myFFT(binVec);
      // Light Measures
      short lightLevel = myLux.lightStrengthLux();

      done_TPL();

      int err=0;
      modem.beginPacket();
      modem.write(inHum);
      modem.write(inTemp);
      modem.write(tempA);
      modem.write(tempB);
      modem.write(HiveWeight);
      modem.write(batteryVoltage);  
      modem.write(lightLevel);
      for(int i = 0; i < 10; i++) {
        modem.write(binVec[i]);
      }
      modem.write(outTemp);
      modem.write(outHum);

      err = modem.endPacket();
      if ( err <= 0 ) {
        // Serial.print("Error : ");
        // Serial.println(err);
        // Confirmation not received - jam or coverage fault
        err_count++;
        if ( err_count > 50 ) {
          State = CONNECTING;
        }
      }
      State = SLEEP;
      break;
    }
    
    case SLEEP:
    {
      done_TPL();
      // Serial.println("DONE !");
      LowPower.sleep(sleep_time_ms - WAKEUP_DELAY_MS);
      State = WAKING;
      break;
    }
  }
}
