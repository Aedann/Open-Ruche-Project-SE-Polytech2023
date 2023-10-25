const int BATTERYPIN = A0; //pin de la batterie

const float TensionMin = 2.5; //tension min
const float TensionMax = 3.3; //tension max

void setup() {
  Serial.begin(9600);
  }

 void loop() { 
    // Lecture de la tension de la batterie sur une broche analogique (par exemple A0) 
    int batteryValue = analogRead(BATTERYPIN); 
    
    // Convertit la valeur analogique en tension (supposons que vous utilisez une référence de 3.3V) 
    float voltage = (batteryValue / 1023.0) * 3.3; 
    
    // Calcule la charge de la batterie en pourcentage 
    float batteryCharge = (voltage - TensionMin) / (TensionMax - TensionMin) * 100;
    
    // Affiche la charge de la batterie sur le moniteur série 
    Serial.print("Tension de la batterie : "); Serial.print(voltage); 
    Serial.println(" V"); 
    Serial.print("Charge de la batterie : "); Serial.print(batteryCharge);
    Serial.println("%"); delay(1000); // Délai d'une seconde
 
 }
