#include <inttypes.h>

#define stringMaxLenght 8;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

      // variables to hold the parsed data
char messageFromPC[numChars] = {0};
uint32_t integerFromPC = 0;
float floatFromPC = 0.0;

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

boolean newData = false;

//============

void setup() {
    Serial.begin(9600);
    Serial1.begin(9600);
    Serial.println("This demo expects 3 pieces of data - text, an integer and a floating point value");
    Serial.println("Enter data in this style <HelloWorld, 12, 24.7>  ");
    Serial.println();
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }
}

//============

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

//============

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

//============

void showParsedData() {
    Serial.print("SendingData.NoObjects :");
    Serial.println(SendingData.NoObjects);
    Serial.print("SendingData.label1 :");
    Serial.println(SendingData.label1);
    Serial.print("SendingData.value1 :");
    Serial.println(SendingData.value1);
    Serial.print("SendingData.x1 :");
    Serial.println(SendingData.x1);
    Serial.print("SendingData.y1 :");
    Serial.println(SendingData.y1);
    Serial.print("SendingData.width1 :");
    Serial.println(SendingData.width1);
    Serial.print("SendingData.height1 :");
    Serial.println(SendingData.height1);
    Serial.print("SendingData.label2 :");
    Serial.println(SendingData.label2);
    Serial.print("SendingData.value2 :");
    Serial.println(SendingData.value2);
    Serial.print("SendingData.x2 :");
    Serial.println(SendingData.x2);
    Serial.print("SendingData.y2 :");
    Serial.println(SendingData.y2);
    Serial.print("SendingData.width2 :");
    Serial.println(SendingData.width2);
    Serial.print("SendingData.height2 :");
    Serial.println(SendingData.height2);
    Serial.print("SendingData.DSP :");
    Serial.println(SendingData.DSP);
    Serial.print("SendingData.Classification :");
    Serial.println(SendingData.Classification);
    Serial.print("SendingData.Anomaly :");
    Serial.println(SendingData.Anomaly);
    Serial.print("SendingData.errCode1 :");
    Serial.println(SendingData.errCode1);
    Serial.print("SendingData.errCode2 :");
    Serial.println(SendingData.errCode2);
    Serial.print("SendingData.errCode3 :");
    Serial.println(SendingData.errCode3);
    Serial.print("SendingData.errCode4 :");
    Serial.println(SendingData.errCode4);
}