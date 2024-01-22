The AI dataset has been completely reworked from scratch, with the second object being the bee barriers that defend the hive. This is because it is an even more significant element of hornet attacks. Hornets can move out of the camera's field every 10 minutes, eluding the AI at the moment it takes a photo. In contrast, the bee barrier remains in front of the hive for the duration of the attack.

The data received by the Arduino board is structured as follows:
```c++
struct SendingData_t {
    uint32_t NoObjects;         // 1 if there is no hornet attack, 0 otherwise
    char label1[numChars];      // Name of the first detected object (Hornet or Bees)
    float value1;               // Confidence percentage
    uint32_t x1;                // x-coordinate of the bounding box
    uint32_t y1;                // y-coordinate of the bounding box
    uint32_t width1;            // Width coordinate of the bounding box
    uint32_t height1;           // Height coordinate of the bounding box
    char label2[numChars];      // Name of the second detected object (Hornet or Bees)
    float value2;               // Confidence percentage
    uint32_t x2;                // x-coordinate of the bounding box
    uint32_t y2;                // y-coordinate of the bounding box
    uint32_t width2;            // Width coordinate of the bounding box
    uint32_t height2;           // Height coordinate of the bounding box
    uint32_t DSP;               // Duration of Digital Signal Processing
    uint32_t Classification;    // Duration of classification calculation
    uint32_t Anomaly;           // Duration of anomaly rate calculation
    uint32_t errCode1;          // See ./Doc_SendingData_t.txt
    uint32_t errCode2;          // See ./Doc_SendingData_t.txt
    uint32_t errCode3;          // See ./Doc_SendingData_t.txt
    uint32_t errCode4;          // See ./Doc_SendingData_t.txt
};
```
To implement image processing, you need to load esp32main.ino onto the ESP32, adding the library ei-hornetandbees-arduino-1.0.2.zip.

Due to a "Camera init failed" problem, I couldn't test it in real-time.

