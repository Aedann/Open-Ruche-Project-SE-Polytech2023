Le dataset de l'IA a été refait de A à Z en prenant comme 2e objet les barrières d'abeilles qui défendent l'entree de la ruche, car c'est un element encore plus signifiant de l'attaque de frelons. Le frelon peut sortir du champ de la caméra toutes les 10 minutes et échapper à la l'IA au moment où il prend sa photo, tandis que la barriere d'abeille reste devant la ruche le temps de l'attaque. 

Les données reçus par la carte arduino sont sous cette forme: 
```c++
struct SendingData_t{
    uint32_t NoObjects;         //1 s'il n'y a pas d'attaque de frelon et 0 sinon
    char label1[numChars];      //Nom du premier objet detecté (Hornet ou Bees)
    float value1;               //Pourcentage de certitude
    uint32_t x1;                //Coordonnée en x de la bounding_boxes 
    uint32_t y1;                //Coordonnée en y de la bounding_boxes 
    uint32_t width1;            //Coordonnée en w de la bounding_boxes 
    uint32_t height1;           //Coordonnée en h de la bounding_boxes 
    char label2[numChars];      //Nom du deuxième objet detecté (Hornet ou Bees)
    float value2;               //Pourcentage de certitude
    uint32_t x2;                //Coordonnée en x de la bounding_boxes 
    uint32_t y2;                //Coordonnée en y de la bounding_boxes 
    uint32_t width2;            //Coordonnée en w de la bounding_boxes 
    uint32_t height2;           //Coordonnée en h de la bounding_boxes 
    uint32_t DSP;               //Durée de la Traitement du signal digital 
    uint32_t Classification;    //Durée du calcul de classification
    uint32_t Anomaly;           //Durée du calcul du taux d'anomalie
    uint32_t errCode1;          //Voir Doc_SendingData_t.txt
    uint32_t errCode2;          //Voir Doc_SendingData_t.txt
    uint32_t errCode3;          //Voir Doc_SendingData_t.txt
    uint32_t errCode4;          //Voir Doc_SendingData_t.txt
};
```
Pour mettre en place le traitement d'image, il faut charger esp32main.ino sur l'esp32 en ajoutant la library ei-hornetandbees-arduino-1.0.2.zip.

A cause d'un probleme "Camera init failed" j'ai pas pu tester en réel
