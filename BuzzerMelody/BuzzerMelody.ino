#include "pitches.h"

int buzzerPin = 10; // Connectez le buzzer au port analogique 10

int melody[] = {
  REST, REST, REST, NOTE_G4, REST, NOTE_G4, REST, 
  NOTE_F4, REST, NOTE_F4, REST, NOTE_D4, REST, 
  NOTE_C4, REST, NOTE_A4, REST
};

int noteDurations[] = {
  9, 9, 9, 6, 6, 6, 6, 
  6, 2, 10, 10, 10, 10, 
  10, 10, 10, 8
};

void setup() {
  // Aucune initialisation spécifique nécessaire
}

void loop() {
  for (int thisNote = 0; thisNote < 18; thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(buzzerPin);
    delay(50); // Ajoutez une petite pause entre les notes
  }
  delay(1000); // Attendez une seconde avant de rejouer la mélodie

  // Changez la mélodie en inversant l'ordre des notes
  /*for (int thisNote = 7; thisNote >= 0; thisNote--) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(buzzerPin, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    noTone(buzzerPin);
    delay(50); // Ajoutez une petite pause entre les notes
  }*/
  delay(1000); // Attendez une seconde avant de rejouer la mélodie
}
