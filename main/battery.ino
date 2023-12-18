#define MAXVOLTAGE 3.3
#define MINVOLTAGE 2.5

const int BATTERY_PIN = A0;

short read_battery_voltage() {
      analogReadResolution(10);
      analogReference(AR_INTERNAL2V23);
      int batteryValue = analogRead(BATTERY_PIN);
      float batteryTemp = (batteryValue / 1023.0) * 3.3;
      // short batteryCharge = (float)((batteryTemp - MINVOLTAGE)/(MAXVOLTAGE - MINVOLTAGE) * 100);
      return (batteryTemp*100);
}