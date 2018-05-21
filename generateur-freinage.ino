// Générateur de freinage
// PBA 2018-05-19

#include <FlexiTimer2.h>

#define DCC_OUT1 2   // Sortie du signal DCC

#define DCC_BIT_HIGH 0 // Les quatre états de l'automate bit
#define DCC_BIT_HIGH0 1
#define DCC_BIT_LOW 2
#define DCC_BIT_LOW0 3

#define DCC_PACKET_IDLE 0 // Les cinq états de l'automate paquet
#define DCC_PACKET_HEADER 1
#define DCC_PACKET_START 2
#define DCC_PACKET_BYTE 3
#define DCC_PACKET_STOP 4

#define DCC_PACKET_SIZE 6 // Taille maximum d'un paquet DCC
#define DCC_HEADER_SIZE 20

byte DccBit; // Bit en cours d'envoi
byte DccSubBit; // Partie du bit en cours d'envoi
byte DccDataMode;    // Variable d'état de l'automate paquet
byte DccPacketsUsed=1; // Nombre de paquets à envoyer  
byte DccHeaderCount; // Comptage des bits à un du préambule
byte DccByteCount;   // Index de l'octet en cours d'envoi
byte DccBitShift;    // Comptage des bits de l'octet à envoyer

byte packetData[DCC_PACKET_SIZE]= // Paquet de données à envoyer
{0,0x40,0x40}; // Commande d'arrêt généralisé

byte packetSize=3; // Taille du paquet à envoyer

void dccInterrupt(void) 
{
  switch(DccSubBit) // Automate bit
  {
  case DCC_BIT_HIGH :
    switch(DccDataMode) // Automate paquet
    {
    case DCC_PACKET_IDLE :
      if(DccPacketsUsed)
      {
        DccDataMode=DCC_PACKET_HEADER;
        DccHeaderCount=DCC_HEADER_SIZE;
      }
      break;
    case DCC_PACKET_HEADER :
      DccBit=1;
      if(!--DccHeaderCount)
      {
        DccDataMode=DCC_PACKET_START;
        DccByteCount=0;
      }
      break;
    case DCC_PACKET_START :
      DccBit=0;
      DccBitShift=0x80;
      DccDataMode=DCC_PACKET_BYTE;
      break;
    case DCC_PACKET_BYTE :
      DccBit=!!(packetData[DccByteCount]&DccBitShift);
      DccBitShift>>=1;
      if(!DccBitShift)
      {
        if(packetSize==++DccByteCount)  // Fin du paquet
          DccDataMode=DCC_PACKET_STOP;
        else
          DccDataMode=DCC_PACKET_START;
      }
      break;
    case DCC_PACKET_STOP :
      DccBit=1;
      DccDataMode=DCC_PACKET_HEADER;
      DccHeaderCount=DCC_HEADER_SIZE;
      break;
    }
    digitalWrite(DCC_OUT1,HIGH);
    if(DccBit)
      DccSubBit=DCC_BIT_LOW;
    else
      DccSubBit=DCC_BIT_HIGH0;
    break;
  case DCC_BIT_HIGH0 :
    digitalWrite(DCC_OUT1,HIGH);
    DccSubBit=DCC_BIT_LOW;
    break;
  case DCC_BIT_LOW :
    digitalWrite(DCC_OUT1,LOW);
    if(DccBit)
      DccSubBit=DCC_BIT_HIGH;
    else
      DccSubBit=DCC_BIT_LOW0;
    break;
  case DCC_BIT_LOW0 :
    digitalWrite(DCC_OUT1,LOW);
    DccSubBit=DCC_BIT_HIGH;
    break;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  FlexiTimer2::set(1, 0.000056, dccInterrupt);
  FlexiTimer2::start();
}

void loop()
{
  delay(1000);
}

