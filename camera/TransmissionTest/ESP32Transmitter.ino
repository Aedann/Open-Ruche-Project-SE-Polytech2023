#include <Arduino.h>

#define DATA_SIZE 2   // 125 bytes is a bit higher than the default 120 bytes of RX FIFO FULL 
#define BAUD 9600       // Any baudrate from 300 to 115200
#define TEST_UART 1     // Serial1 will be used for the loopback testing with different RX FIFO FULL values
#define RXPIN 4         // GPIO 4 => RX for Serial1
#define TXPIN 5         // GPIO 5 => TX for Serial1

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing


struct SendingData_t{
    uint32_t NoObjects;
    char label1[numChars];
    float value1;
    uint32_t x1;
    uint32_t y1;
    uint32_t width1;
    uint32_t height1;
    char label2[numChars];
    float value2;
    uint32_t x2;
    uint32_t y2;
    uint32_t width2;
    uint32_t height2;
    uint32_t DSP;
    uint32_t Classification;
    uint32_t Anomaly;
    uint32_t errCode1;
    uint32_t errCode2;
    uint32_t errCode3;
    uint32_t errCode4;
};


struct SendingData_t SendingData = {0,"Hornet", 0.66, 314,15,9,26,"Bees", 0.66, 314,15,9,26,45,90,180,404,200,301,501};

//struct SendingData_t SendingData = {0,"Test",0.0,0,0,0,0,"Test2",0.0,0,0,0,0};
void setup() {
  //Serial1.begin(9600);
  Serial.begin(9600);

}

void loop() {
  // Le reste du code...
    //sentBytes = Serial1.write(dataSent, DATA_SIZE);

     //Correction du format de spécificateur pour imprimer un uint8_t
  char charToSend[300];  // Ajustez la taille selon vos besoins
  snprintf(charToSend, sizeof(charToSend), "<%u,%s, %.2f, %u,%u,%u,%u,%s, %.2f, %u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u>",
            SendingData.NoObjects,
             SendingData.label1, SendingData.value1,
            SendingData.x1, SendingData.y1, SendingData.width1, SendingData.height1,
             SendingData.label2, SendingData.value2,
            SendingData.x2, SendingData.y2, SendingData.width2, SendingData.height2,
            SendingData.DSP,SendingData.Classification,SendingData.Anomaly,
            SendingData.errCode1,SendingData.errCode2,SendingData.errCode3,SendingData.errCode4);

  Serial.printf(charToSend);
  delay(500);
  
}
