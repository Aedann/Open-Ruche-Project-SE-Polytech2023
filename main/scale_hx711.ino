#include "HX711.h"

#define SCALE_MEANSF 286.53
#define SCALE_METALZERO 1264.7
const int LOADCELL_DOUT_PIN = 9;
const int LOADCELL_SCK_PIN = 8;
HX711 scale;

void init_HX711() {
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale.set_scale(SCALE_MEANSF);
  // Serial.println("HX711 init'd!");
}

short getWeight() {
  return (short)(scale.get_units(10) - SCALE_METALZERO);
}