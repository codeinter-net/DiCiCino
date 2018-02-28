// PCF8591TEST - TESTER FOR I²C ADC CIRCUIT
// send and receive packets and check for errors
// PBA 2017-02-28

#include<Wire.h>
#define NUM_CONTROL 8 // Nombre de contrôleurs
#define DATASIZE 4    // 4 octets à lire

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Wire.begin();
}

void loop()
{
  byte i;
  for(i=0;i<NUM_CONTROL;i++)
  {
    Serial.print(i);             // Affiche le numéro du contrôleur
    Serial.print(" : ");
    Wire.beginTransmission(0x48+i) ;// S'adresse au PCF8591
    Wire.write(0x45);            // Commande
    Wire.write(0x00);            // Donnée vide envoyée au DAC
    Wire.endTransmission();      // effectue l'envoi et termine
  
    Wire.requestFrom(0x48+i,DATASIZE); // Demande la lecture
    while(Wire.available())      // Tant que des données sont disponibles
    {
      Serial.print(Wire.read()); // Lecture et affichage
      Serial.write(' ');
    }
    Serial.println("");
  }
  delay(1000); // on attend
}

