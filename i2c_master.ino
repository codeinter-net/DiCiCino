// I²C MASTER - TESTING I²C BUS
// send and receive packets and check for errors
// PBA 2017-02-10

#include<Wire.h>
#define DATASIZE 4 // On se limite à des trames de 4 octets

void setup()
{
  Serial.begin(9600);
  while(!Serial);
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // La LED signale le résultat
}

void loop()
{
  // Prépare les données à envoyer
  byte writeData[DATASIZE];
  byte readData[DATASIZE];
  unsigned long micSec = micros(); // le compteur de microsecondes
  byte i;
  for(i=0; i<DATASIZE; i++)
  {
    writeData[i]=micSec&0xFF; // les microsecondes servent de données
    micSec>>=8;
  }
  
  // Envoi des données
  Wire.beginTransmission(0x70) ; // S'adresse au périphérique 0x70 
  Wire.write(0x22);
  Wire.write(sizeof(writeData));
  Wire.write(writeData, sizeof(writeData));
  Wire.endTransmission(); // effectue l'envoi et termine
  delay(500); // on attend

  // Réception des données
  digitalWrite(LED_BUILTIN, HIGH); // Pour faire un flash si OK
  Wire.beginTransmission(0x70) ; // S'adresse au périphérique 0x70
  Wire.write(0x11);
  Wire.write(sizeof(readData)); // Demande de lecture
  Wire.endTransmission(); // effectue l'envoi et termine
  Wire.requestFrom(0x70,sizeof(readData)); // Attend les données 
  i=0;
  while(Wire.available()) // Tant que des données sont disponibles
  {
    readData[i++]=Wire.read(); // Lit les données
    if(i==sizeof(readData)) break; // Pour éviter un buffer overflow
  }

  // Vérification des données reçues
  if(!memcmp(writeData,readData,DATASIZE)) // Compare les deux trames
  {
    Serial.println("OK");
    digitalWrite(LED_BUILTIN, LOW); // LED éteinte + flash = OK
  }
  else
  {
    Serial.println("ERREUR");
    digitalWrite(LED_BUILTIN, HIGH); // LED allumée = erreur
  }
  delay(500); // on attend
}

