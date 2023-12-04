#include "wiring_private.h"

// ADC-related

#define sampleLimit 128
uint32_t startTime;
const uint8_t plusPin = A6;

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


void setup() {

  Serial.begin(115200);
  delay(2000);

  pinPeripheral(plusPin, PIO_ANALOG);

  ADC->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_INTVCC0_Val;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->CTRLB.bit.RESSEL = 1;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->AVGCTRL.bit.SAMPLENUM = 0x5; 
  while (ADC->STATUS.bit.SYNCBUSY);

  ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV32_Val;
  while (ADC->STATUS.bit.SYNCBUSY) ; // Wait for clock domain sysch

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[plusPin].ulADCChannelNumber;
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

  while (!(ADC->INTFLAG.bit.RESRDY));
  startTime = micros();
  ADC->INTFLAG.bit.RESRDY = 0x1;     // Discard first reading

  ADC->INTENSET.bit.RESRDY = 1;
  NVIC_EnableIRQ(ADC_IRQn);
  Serial.println("Starting ...");
}

//uint32_t stopTime, sample;
volatile uint32_t sampleCount = 0, stopTime;
volatile int32_t runningSum = 0, lastReading;
volatile uint8_t doneFlag = 0;

volatile int16_t sampleVec[sampleLimit];

void loop() {
  int32_t avgReading;
  float avgSamplePeriod;
  
  if (doneFlag) {    
    avgReading = runningSum / (int32_t)sampleCount;
    avgSamplePeriod = (float)(stopTime-startTime)/sampleCount;
    // Serial.print("Samples: ");Serial.print(sampleCount);Serial.print(", Avg Sample Period: ");Serial.print(avgSamplePeriod);Serial.print(" us, Avg Reading = ");Serial.println(avgReading);
    // for(int i = 0; i < sampleLimit; i++) {
    //   Serial.println(sampleVec[i]);
    // }
    Serial.print("Sample period: ");
    Serial.print(avgSamplePeriod);
    Serial.print(" us, total period: ");
    Serial.print(stopTime - startTime);
    Serial.println(" us");
    doneFlag = 0;
    sampleCount = 0;
    runningSum = 0;
    while(ADC->INTFLAG.bit.RESRDY == 0);   // Clean up any samples we missed during printing
    ADC->INTFLAG.bit.RESRDY = 0x1;
    startTime = micros();
    ADC->INTENSET.bit.RESRDY = 1;
    
    for(int i  = 0; i<samples; i++) {
      vReal[i] = sampleVec[i];
      vImag[i] = 0;
    }

    FFT = arduinoFFT(vReal, vImag, samples, samplingFrequency); /* Create FFT object */
    FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);	/* Weigh data */
    FFT.Compute(FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(); /* Compute magnitudes */
    // Serial.println("Computed magnitudes:");
    // PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
    
    short uint bin098_146 = average_bin(11,15);
    short uint bin146_195 = average_bin(16,21);
    short uint bin195_244 = average_bin(22,26);
    short uint bin244_293 = average_bin(27,31);
    short uint bin293_342 = average_bin(32,37);
    short uint bin342_391 = average_bin(38,42);
    short uint bin391_439 = average_bin(43,47);
    short uint bin439_488 = average_bin(48,53);
    short uint bin488_537 = average_bin(53,58);
    short uint bin537_581 = average_bin(59,63);

    Serial.println(bin098_146);
    Serial.println(bin146_195);
    Serial.println(bin195_244);
    Serial.println(bin244_293);
    Serial.println(bin293_342);
    Serial.println(bin342_391);
    Serial.println(bin391_439);
    Serial.println(bin439_488);
    Serial.println(bin488_537);
    Serial.println(bin537_581);

    while(1);
  }
}

short uint average_bin(int b, int e) {
  double sum = 0;
  for(int i = b; i <= e; i++) sum =+ vReal[i];
  return (short uint) (sum / (float)(e-b));
}

void ADC_Handler() {
  uint32_t sample;
  int32_t signedSample;

  sample = ADC->RESULT.reg;
  // if (sample & 0x800) {
  //   sample |= 0xfffff000;
  // }
  signedSample = (int32_t) sample;
  // runningSum += signedSample;
  sampleVec[sampleCount] = signedSample;
  ADC->INTFLAG.bit.RESRDY = 0x1;
  if(++sampleCount >= sampleLimit) {
    ADC->INTENCLR.bit.RESRDY = 1;
    stopTime = micros();
    lastReading = signedSample;
    doneFlag = 1;
  }
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
	break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
	break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
	break;
    }
    Serial.print(i);
    Serial.print(" - ");
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}
