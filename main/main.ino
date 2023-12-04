#include <Arduino.h>
#include <MKRWAN.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_B_LUX_V30B.h>
#include <ArduinoLowPower.h>
#include "HX711.h"
#include "DHT22.h"

// Serial-related DEFINEs

#define SERIAL_BAUDRATE 9600

// DHT-related
const int DHT_PIN = 1;
DHT22 dht22(DHT_PIN);

// OneWire Temp Probe-related
const int ONEWIRE_PIN_A = 2;
const int ONEWIRE_PIN_B = 3;
OneWire oneWire_a(ONEWIRE_PIN_A);
OneWire oneWire_b(ONEWIRE_PIN_B);
DallasTemperature probe_a(&oneWire_a);
DallasTemperature probe_b(&oneWire_b);

// Scale-related
#define SCALE_MEANSF 286.53
#define SCALE_METALZERO 1258.7
const int LOADCELL_DOUT_PIN = 9;
const int LOADCELL_SCK_PIN = 8;
HX711 scale;

// Battery-related
#define MAXVOLTAGE 3.3
#define MINVOLTAGE 2.5
const int BATTERY_PIN = A0;

//Audio-related
// #include "wiring_private.h"
// #define sampleLimit 128
// uint32_t startTime;
// const int MIC_IN_PIN = A6;

// #include "arduinoFFT.h"
// arduinoFFT FFT;
// const uint16_t samples = sampleLimit; //This value MUST ALWAYS be a power of 2
// const double samplingFrequency = 1180.6375;
// double vReal[samples];
// double vImag[samples];
// #define SCL_INDEX 0x00
// #define SCL_TIME 0x01
// #define SCL_FREQUENCY 0x02
// #define SCL_PLOT 0x03
// volatile uint32_t sampleCount = 0, stopTime;
// volatile int32_t runningSum = 0, lastReading;
// volatile uint8_t doneFlag = 0;
// volatile int16_t sampleVec[sampleLimit];
// short unsigned binVec[10];

//Luxmeter-related
DFRobot_B_LUX_V30B myLux(0);

//Loadswitch-related
const int LOADSWITCH_DELAY_PIN = 7;
const int LOADSWITCH_DONE_PIN = 6;

// Power-related
#define WAKEUP_DELAY_MS 2000;
int sleep_time_ms = 20000;

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

void init_OneWire() {
  probe_a.begin();
  probe_b.begin();
}

void init_HX711() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(SCALE_MEANSF);
}

void init_Lux() {
  myLux.begin();
}

// void init_ADC() {

//   pinPeripheral(MIC_IN_PIN, PIO_ANALOG);

//   ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->CTRLB.bit.RESSEL = 1;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->AVGCTRL.bit.SAMPLENUM = 0x5; 
//   while (ADC->STATUS.bit.SYNCBUSY);

//   ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[MIC_IN_PIN].ulADCChannelNumber;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->INPUTCTRL.bit.MUXNEG = 0x19;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_2X_Val;

//   ADC->CTRLB.bit.DIFFMODE = 0x1;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch
  
//   ADC->CTRLB.bit.FREERUN = 0x1;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->CTRLA.bit.ENABLE = 0x1;
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->INTFLAG.bit.RESRDY = 0x1;     // Clear ready flag
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   ADC->SWTRIG.bit.START = 1;         // Start ADC
//   while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

//   while (!(ADC->INTFLAG.bit.RESRDY));
//   startTime = micros();
//   ADC->INTFLAG.bit.RESRDY = 0x1;     // Discard first reading

//   ADC->INTENSET.bit.RESRDY = 1;
//   NVIC_EnableIRQ(ADC_IRQn);
// }

// void ADC_Handler() {
//   uint32_t sample;
//   int32_t signedSample;

//   sample = ADC->RESULT.reg;
//   // if (sample & 0x800) {
//   //   sample |= 0xfffff000;
//   // }
//   signedSample = (int32_t) sample;
//   // runningSum += signedSample;
//   sampleVec[sampleCount] = signedSample;
//   ADC->INTFLAG.bit.RESRDY = 0x1;
//   if(++sampleCount >= sampleLimit) {
//     ADC->INTENCLR.bit.RESRDY = 1;
//     stopTime = micros();
//     lastReading = signedSample;
//     doneFlag = 1;
//   }
// }

// void myFFT(short unsigned bintab[10])  {
//   int32_t avgReading;
//   float avgSamplePeriod;
  
//   if (doneFlag) {    
//     avgReading = runningSum / (int32_t)sampleCount;
//     avgSamplePeriod = (float)(stopTime-startTime)/sampleCount;
//     // Serial.print("Samples: ");Serial.print(sampleCount);Serial.print(", Avg Sample Period: ");Serial.print(avgSamplePeriod);Serial.print(" us, Avg Reading = ");Serial.println(avgReading);
//     // for(int i = 0; i < sampleLimit; i++) {
//     //   Serial.println(sampleVec[i]);
//     // }
//     Serial.print("Sample period: ");
//     Serial.print(avgSamplePeriod);
//     Serial.print(" us, total period: ");
//     Serial.print(stopTime - startTime);
//     Serial.println(" us");
//     doneFlag = 0;
//     sampleCount = 0;
//     runningSum = 0;
//     while(ADC->INTFLAG.bit.RESRDY == 0);   // Clean up any samples we missed during printing
//     ADC->INTFLAG.bit.RESRDY = 0x1;
//     startTime = micros();
//     ADC->INTENSET.bit.RESRDY = 1;
    
//     for(int i  = 0; i<samples; i++) {
//       vReal[i] = sampleVec[i];
//       vImag[i] = 0;
//     }

//     FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
//     FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
//     FFT.Compute(FFT_FORWARD); /* Compute FFT */
//     FFT.ComplexToMagnitude(); /* Compute magnitudes */
//     // Serial.println("Computed magnitudes:");
//     // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
    
//     bintab[0] = average_bin(11,15); //bin098_146
//     bintab[1] = average_bin(16,21); //bin146_195
//     bintab[2] = average_bin(22,26); //bin195_244
//     bintab[3] = average_bin(27,31); //bin244_293
//     bintab[4] = average_bin(32,37); //bin293_342
//     bintab[5] = average_bin(38,42); //bin342_391
//     bintab[6] = average_bin(43,47); //bin391_439
//     bintab[7] = average_bin(48,53); //bin439_488
//     bintab[8] = average_bin(53,58); //bin488_537
//     bintab[9] = average_bin(59,63); //bin537_581

//     // Serial.println(bin098_146);
//     // Serial.println(bin146_195);
//     // Serial.println(bin195_244);
//     // Serial.println(bin244_293);
//     // Serial.println(bin293_342);
//     // Serial.println(bin342_391);
//     // Serial.println(bin391_439);
//     // Serial.println(bin439_488);
//     // Serial.println(bin488_537);
//     // Serial.println(bin537_581);

//     while(1);
//   }
// }

// short unsigned average_bin(int b, int e) {
//   double sum = 0;
//   for(int i = b; i <= e; i++) sum =+ vReal[i];
//   return (short unsigned) (sum / (float)(e-b));
// }

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
  wake_TPL();
  delay(1000);

  init_serial();
  init_WAN();
  init_OneWire();
  // init_HX711();
  // init_ADC();
  // init_Lux();
  beep();

  Serial.println("INIT !");

  State = CONNECTING;
  delay(1000);
}

void loop() {
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
        State = WAKING;
      }
      break;
    }

    case WAKING:
    {
      wake_TPL();
      Serial.println("WAKING UP !");
      
      delay(1000);
      State = SENDING;

      break;
    }

    case SENDING:
    {
      // DHT Measures
      short inTemp = ((float)dht22.getTemperature())*10;
      short inHum = ((float)dht22.getHumidity())*10;
      // OneWire Measures
      probe_a.requestTemperatures();
      probe_b.requestTemperatures();
      short tempA = ((float)probe_a.getTempCByIndex(0)*10);
      short tempB = ((float)probe_b.getTempCByIndex(0)*10);
      // Scale Measures
      short HiveWeight = (scale.get_units(10) - SCALE_METALZERO);
      // Battery Measures
      float batteryTemp = analogRead(BATTERY_PIN);
      batteryTemp = (batteryTemp / 1023.0) * 3.3;
      short batteryCharge = (float)((batteryTemp - MINVOLTAGE)/(MAXVOLTAGE - MINVOLTAGE) * 100);
      short batteryVoltage = batteryTemp*100;
      // Audio Measures
      // myFFT(binVec);
      // Light Measures
      // short lightLevel = myLux.lightStrengthLux();

      done_TPL();

      int err=0;
      modem.beginPacket();
      modem.write(inHum);
      modem.write(inTemp);
      modem.write(tempA);
      modem.write(tempB);
      modem.write(HiveWeight);
      modem.write(batteryVoltage);  
      // modem.write(lightLevel);
      // for(int i = 0; i < 10; i++) {
      //   modem.write(binVec[i]);
      // }

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
      State = SLEEP;
      break;
    }
    
    case SLEEP:
    {
      done_TPL();
      Serial.println("DONE !");
     // LowPower.sleep(20000);
      LowPower.sleep(sleep_time_ms);
      State = WAKING;
      break;
    }
  }
}
