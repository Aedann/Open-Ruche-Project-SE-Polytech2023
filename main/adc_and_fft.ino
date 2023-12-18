#include "wiring_private.h"
#include "ArduinoLowPower.h"

// ADC-related

#define sampleLimit 128
const uint8_t MIC_IN_PIN = A6;

uint32_t startTime;
uint32_t sampleCount = 0, stopTime;
int32_t runningSum = 0, lastReading;
volatile uint8_t doneFlag = 0;
volatile int16_t sampleVec[sampleLimit];

// FFT-related

#include "arduinoFFT.h"
arduinoFFT FFT;
const uint16_t samples = sampleLimit; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 1180.6375;
double vReal[samples];
double vImag[samples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03


void init_ADC() {

  pinPeripheral(MIC_IN_PIN, PIO_ANALOG);

  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->CTRLB.bit.RESSEL = 1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->AVGCTRL.bit.SAMPLENUM = 0x5; 
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[MIC_IN_PIN].ulADCChannelNumber;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INPUTCTRL.bit.MUXNEG = 0x19;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INPUTCTRL.bit.GAIN = ADC_INPUTCTRL_GAIN_2X_Val;

  ADC->CTRLB.bit.DIFFMODE = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch
  
  ADC->CTRLB.bit.FREERUN = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->CTRLA.bit.ENABLE = 0x1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INTFLAG.bit.RESRDY = 0x1;     // Clear ready flag
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->SWTRIG.bit.START = 1;         // Start ADC
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  while (!(ADC->INTFLAG.bit.RESRDY)); // Wait for first reading
  startTime = micros();
  ADC->INTFLAG.bit.RESRDY = 0x1;     // Discard first reading

  ADC->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ(ADC_IRQn);
  // Serial.println("Starting ADC ...");
}

void myFFT(short unsigned binVec[10]) {
  int32_t avgReading;
  float avgSamplePeriod;

  // Serial.println("Starting myFFT...");
  doneFlag = 0;
  sampleCount = 0;

  init_ADC();

  while(!doneFlag);
  // Serial.println("Done recording.");

  if(doneFlag) {
    avgReading = runningSum / (int32_t)sampleCount;
    avgSamplePeriod = (float)(stopTime-startTime)/sampleCount;
    // Serial.print("Sample period: ");
    // Serial.print(avgSamplePeriod);
    // Serial.print(" us, total period: ");
    // Serial.print(stopTime - startTime);
    // Serial.println(" us");
    doneFlag = 0;
    sampleCount = 0;
    // Serial.println("testRESRDY");
    while(ADC->INTFLAG.bit.RESRDY == 0);   // Clean up any samples we missed during printing
    // Serial.println("DoneRESRDY");
    ADC->INTFLAG.bit.RESRDY = 0x1;
    startTime = micros();
    ADC->INTENSET.bit.RESRDY = 1;
    
    for(int i  = 0; i<samples; i++) {
      vReal[i] = sampleVec[i];
      vImag[i] = 0;
    }
    // Serial.println("Done_vReal");

    FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(); /* Compute magnitudes */

    // Serial.println("Done FFT");
    
    binVec[0] = average_bin(11,15);
    binVec[1] = average_bin(16,21);
    binVec[2] = average_bin(22,26);
    binVec[3] = average_bin(27,31);
    binVec[4] = average_bin(32,37);
    binVec[5] = average_bin(38,42);
    binVec[6] = average_bin(43,47);
    binVec[7] = average_bin(48,53);
    binVec[8] = average_bin(53,58);
    binVec[9] = average_bin(59,63);
  }
}

short average_bin(int b, int e) {
  double sum = 0;
  for(int i = b; i <= e; i++) sum =+ vReal[i];
  return (short) (sum / (float)(e-b));
}

void ADC_Handler() {
  uint32_t sample;
  int32_t signedSample;

  sample = ADC->RESULT.reg;
  // if (sample & 0x800) {
  //   sample |= 0xfffff000;
  // }
  signedSample = (int32_t) sample;
  sampleVec[sampleCount] = signedSample;
  ADC->INTFLAG.bit.RESRDY = 0x1;
  if(++sampleCount >= sampleLimit) {
    ADC->INTENCLR.bit.RESRDY = 1;
    stopTime = micros();
    lastReading = signedSample;
    doneFlag = 1;
    // Serial.println("Finished recording.");
  }
}

// void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
// {
//   for (uint16_t i = 0; i < bufferSize; i++)
//   {
//     double abscissa;
//     /* Print abscissa value */
//     switch (scaleType)
//     {
//       case SCL_INDEX:
//         abscissa = (i * 1.0);
// 	break;
//       case SCL_TIME:
//         abscissa = ((i * 1.0) / samplingFrequency);
// 	break;
//       case SCL_FREQUENCY:
//         abscissa = ((i * 1.0 * samplingFrequency) / samples);
// 	break;
//     }
//     Serial.print(i);
//     Serial.print(" - ");
//     Serial.print(abscissa, 6);
//     if(scaleType==SCL_FREQUENCY)
//       Serial.print("Hz");
//     Serial.print(" ");
//     Serial.println(vData[i], 4);
//   }
//   Serial.println();
// }
