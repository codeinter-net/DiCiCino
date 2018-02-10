// I²C SLAVE - TESTING I²C BUS
// receive, store and send packets
// PBA 2017-02-10

#include<Wire.h>
#define DATASIZE 16 // Trames de 16 octets maximum

byte data[DATASIZE]; // stockage des données reçues
byte cmd,dataSize;  //

void setup()
{
  Wire.begin(0x70); // Initialisation en esclave sur l'adresse 0x70
  Wire.onReceive(wireReceiveEventFunction); // Événement réception
  Wire.onRequest(wireRequestEventFunction); // Événement émission
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // La LED signale le transfert
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  delay(1000); // Rien à faire à part les événements
}

void wireReceiveEventFunction() // Fonction de réception
{
  byte i;
  digitalWrite(LED_BUILTIN, HIGH); // LED allumée = réception
  while(!Wire.available()); // Attend les données
  cmd = Wire.read(); // Lecture de la commande
  while(!Wire.available()); // Attend les données
  dataSize = Wire.read(); // lecture de la taille des données
  if(dataSize>DATASIZE) dataSize=DATASIZE; // Pour éviter un buffer overflow
  switch(cmd)
  {
    case 0x11 : // lecture
      break; // En cas de lecture, tout est fait dans wireRequestEventFunction
    case 0x22 : // écriture
      i=0;
      while(Wire.available()) // Tant que des données sont disponibles
      {
        data[i++]=Wire.read(); // Lecture des données
        if(i==dataSize) break;
      }
      break; 
    default :
      cmd = 0; // commande inconnue
  }
} 

void wireRequestEventFunction() // Fonction d'émission
{
  digitalWrite(LED_BUILTIN, LOW); // LED éteinte = émission
  switch(cmd)
  {
    case 0x11 : // lecture
      Wire.write(data,dataSize);
      break;
    case 0x22 : // écriture
      break;   // rien à faire
  }
}

