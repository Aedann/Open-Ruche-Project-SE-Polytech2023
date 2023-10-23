#include <Wire.h>

#define TVL_RST_PIN;

void W(char r, char d) {
  Wire.write(0x30);
  Wire.write(r);
  Wire.write(d);
}

void setup() {
  Wire.begin(8);
  Serial.begin(9600);

  //DRIVE RESET HIGH FOR AT LEAST 10 NS

  // INITIAL CONFIG

  W(0x00,0x00); // set register page to 0
  W(0x01,0x01); // initiate rst switch

  // PLL AND CLOCK CONFIG

  W(0x04,0x00); // both CLKs at BCLK
  W(0x05,0x0D); // PLL unpowered, PLL div at ##, PLL mult at ##
  W(0x06,0x04); // J-Val = 4
  W(0x07,0x00); // D-val = 0
  W(0x08,0x00); // D-val = 0

  // POWER THE PLL ONCE ITS CONFIG IS OVER

  W(0x05,0x8D); // PLL powered
  delay(10); // wait for PLL to start up

  // ADC CONFIG

  W(0x12,0x51); // NADC = 1, divider powered on
  W(0x13,0x52); // MADC = 2, divider powered on

  // OSR CONFIG

  W(0x14,0x50); // AOSR = 128 (default)

  // I²S CONFIG

  W(0x1B,0x00); // I²S mode, 16-bit wordlength, slave mode (default)

  // PROCESSING BLOCK CONFIG

  W(0x3D,0x01); // use PRB_01

  // MIKE INPUT CONFIG

  W(0x00,0x01); // set register page to 1
  W(0x33,0x00); // MICBIAS unused
  W(0x3b,0x00); // Left Analog PGA = 0dB
  W(0x3c,0x00); // Right Analog PGA = 0dB
  W(0x34,0xFC); // Left PGA (IN1L-P) to Left ADC as Single-Ended
  W(0x37,0xFC); // Right PGA (IN1L-N) to Right ADC as Single-Ended

  // FINAL CONFIG

  W(0x00,0x00); // set register page to 0
  W(0x51,0xC2); // power up both ADCs
  W(0x52,0x00); // unmute volume control and set gain to 0

}

void loop() {
  // put your main code here, to run repeatedly:

}