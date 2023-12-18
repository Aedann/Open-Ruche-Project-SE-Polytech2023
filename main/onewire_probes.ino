#include <OneWire.h>
#include <DallasTemperature.h>

const int ONEWIRE_PROBE_PIN = 2;
OneWire oneWire_probes(ONEWIRE_PROBE_PIN);
DallasTemperature temp_probes(&oneWire_probes);

void init_OneWire() {
  temp_probes.begin();
  // Serial.println("OneWire probes init'd!");
}

void oneWire_read_temperatures(short *tempA, short *tempB) {
    temp_probes.requestTemperatures();
    *tempA = (short)((float)temp_probes.getTempCByIndex(0)*10);
    *tempB = (short)((float)temp_probes.getTempCByIndex(1)*10);
}