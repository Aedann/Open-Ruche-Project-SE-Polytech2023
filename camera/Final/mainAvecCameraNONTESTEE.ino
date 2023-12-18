#include <Arduino.h>
#include <MKRWAN.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DFRobot_B_LUX_V30B.h>
#include <ArduinoLowPower.h>
#include "HX711.h"
#include "DHT22.h"
#include <inttypes.h> // Pour la partie caméra

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

// Partie Caméra -----------------------------------------------

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
uint32_t integerFromPC = 0;
float floatFromPC = 0.0;

struct SendingData_t{
    uint32_t NoObjects;         //1 s'il n'y a pas d'attaque de frelon et 0 sinon
    char label1[numChars];      //Nom du premier objet detecté (Hornet ou Bees)
    float value1;               //Pourcentage de certitude
    uint32_t x1;                //Coordonnée en x de la bounding_boxes 
    uint32_t y1;                //Coordonnée en y de la bounding_boxes 
    uint32_t width1;            //Coordonnée en w de la bounding_boxes 
    uint32_t height1;           //Coordonnée en h de la bounding_boxes 
    char label2[numChars];      //Nom du deuxième objet detecté (Hornet ou Bees)
    float value2;               //Pourcentage de certitude
    uint32_t x2;                //Coordonnée en x de la bounding_boxes 
    uint32_t y2;                //Coordonnée en y de la bounding_boxes 
    uint32_t width2;            //Coordonnée en w de la bounding_boxes 
    uint32_t height2;           //Coordonnée en h de la bounding_boxes 
    uint32_t DSP;               //Durée de la Traitement du signal digital 
    uint32_t Classification;    //Durée du calcul de classification
    uint32_t Anomaly;           //Durée du calcul du taux d'anomalie
    uint32_t errCode1;          //Voir Doc_SendingData_t.txt
    uint32_t errCode2;          //Voir Doc_SendingData_t.txt
    uint32_t errCode3;          //Voir Doc_SendingData_t.txt
    uint32_t errCode4;          //Voir Doc_SendingData_t.txt};
};

boolean newData = false;

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}
//Après parseData, on a récupéré les variables sorties par l'IA, en fonction de comment beep peut accepter ces
//valeurs on devra peut etre réécrire cette fonction pour laisser les valeurs en format ASCII.
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    sscanf(strtokIndx,"%"SCNu32,&SendingData.NoObjects); // copy it to SendingData.NoObjects
    strtokIndx = strtok(NULL, ",");
    strcpy(SendingData.label1, strtokIndx); // copy it to SendingData.label1 
    strtokIndx = strtok(NULL, ",");
    SendingData.value1 = atof(strtokIndx);     // copy it to SendingData.value1
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx,"%"SCNu32,&SendingData.x1);     // copy it to SendingData.x1
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx,"%"SCNu32,&SendingData.y1);     // copy it to SendingData.y1
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx,"%"SCNu32,&SendingData.width1);     // copy it to SendingData.width1
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx,"%"SCNu32,&SendingData.height1);     // copy it to SendingData.height1
    strtokIndx = strtok(NULL, ",");
    strcpy(SendingData.label2, strtokIndx);
    strtokIndx = strtok(NULL, ",");
    SendingData.value2 = atof(strtokIndx);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.x2);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.y2);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.width2);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.height2);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.DSP);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.Classification);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.Anomaly);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.errCode1);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.errCode2);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.errCode3);
    strtokIndx = strtok(NULL, ",");
    sscanf(strtokIndx, "%" SCNu32, &SendingData.errCode4);
}

//Pour envoyer sur TTN
void sendStructData(const SendingData_t &data) {
    const uint16_t bufferSize = 256;
    char buffer[bufferSize];

    snprintf(buffer, bufferSize,
             "%u,%s,%.2f,%u,%u,%u,%u,%s,%.2f,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
             data.NoObjects, data.label1, data.value1, data.x1, data.y1, data.width1, data.height1,
             data.label2, data.value2, data.x2, data.y2, data.width2, data.height2, data.DSP,
             data.Classification, data.Anomaly, data.errCode1, data.errCode2, data.errCode3,
             data.errCode4);

    modem.write(buffer, strlen(buffer));
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
  //Partie Caméra 
    Serial1.begin(9600);
    struct SendingData_t SendingData = {1,"", 0.0, 0,0,0,0,"", 0, 0,0,0,0,0,0,0,0,0,0,0};

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
      //Partie Caméra : ------------------------------------------------------------
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData(); //--------------A commenter pour le deploy.
        newData = false;
    }

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
      //Partie Camera ---------------------
      //sendStructData(SendingData); //A Decommenter pour tester l'envoie (par encore parsé sur TTN)

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
