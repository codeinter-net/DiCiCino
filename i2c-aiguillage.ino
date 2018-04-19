// I²C AIGUILLAGE ET ELECTRO-AIMANTS
// Commande d'aiguillage par protocole I²C
// PBA 2018-04-09

#include <Wire.h>
#include <EEPROM.h>

#define DATASIZE 4 // Taille maximum des données à recevoir
#define NUM_AIG 32 // Nombre d'aiguillages à gérer
#define EA_PULSE 50 // Durée de l'impulsion envoyée aux électro-aimants
#define STACK_SIZE 256 // Taille de la pile de stockage

// assignation de la matrice aux sorties de l'Arduino
byte eaRows[8]={2,3,4,5,6,7,8,9};
byte eaCols[8]={10,11,12,13,A0,A1,A2,A3};

byte data[DATASIZE]; // stockage des données reçues
byte cmd,dataSize,stackIn,stackOut;

typedef struct _progStruct
{
  byte dir_col; // Voie directe
  byte dir_row;
  byte dev_col; // Voie déviée
  byte dev_row;
} progStruct;

progStruct progTable[NUM_AIG]; // Table de programmation des aiguillages
byte aigPos[NUM_AIG]; // Position des aiguillages
byte aigStack[STACK_SIZE]; // Stockage des aiguilles à commander

void eaSwitch(byte rowCol) // Bascule un aiguillage
{
  byte row = rowCol>>4;
  byte col = rowCol&0x7;
  digitalWrite(eaRows[row],HIGH); // Activation d'une ligne
  digitalWrite(eaCols[col],HIGH); // et d'une colonne
  delay(EA_PULSE);
  digitalWrite(eaRows[row],LOW);  // Désactivation
  digitalWrite(eaCols[col],LOW);
}

void directSwitch(byte rowCol) // Stocke un ordre dans la pile
{
  aigStack[stackIn++]=rowCol;
// On ne se donne pas la peine de tester le débordement de la pile
// de toute façon des données seront perdues
}

void setPos(byte aigNum,byte dirDev) // Convertit d'adresse de l'aiguillage et stocke l'ordre
{
  byte row,col;
  if(dirDev) // voie déviée
  {
    row = progTable[aigNum].dev_row;
    col = progTable[aigNum].dev_col;
    aigPos[aigNum]=1;
  }
  else // voie directe
  {
    row = progTable[aigNum].dir_row;
    col = progTable[aigNum].dir_col;
    aigPos[aigNum]=0;
  }
  if(!(row&0xF8)&&!(col&0xF8)) // teste sil'aiguillage est programmé
    directSwitch((row<<4)|col);
}

byte getPos(byte cmd) // Retourne la position de l'aiguillage
{
  return aigPos[cmd];
}

void setProg(byte aigNum, byte dir, byte dev) // Change la programmation de l'aiguillage
{
  progTable[aigNum].dev_row = dev>>4;
  progTable[aigNum].dev_col = dev&0xF;
  progTable[aigNum].dir_row = dir>>4;
  progTable[aigNum].dir_col = dir&0xF;

  byte* tablePtr = (byte*)&progTable[aigNum]; // Pointeur un aiguillage dans la table
  int tableIndex = aigNum*sizeof(progStruct);
  byte i;
  for(i=0; i<sizeof(progStruct); i++)
  {
    EEPROM.write(tableIndex++,*tablePtr++); // Copie directe via le pointeur
  }
}

byte getProg(byte aigNum,byte dirDev) // Retourne la programmation de l'aiguillage
{
  byte row,col;
  if(dirDev) // voie déviée
  {
    row = progTable[aigNum].dev_row;
    col = progTable[aigNum].dev_col;
  }
  else // voie directe
  {
    row = progTable[aigNum].dir_row;
    col = progTable[aigNum].dir_col;
  }
  return (row<<4)|col;
}

void setup()
{
  Wire.begin(0x71); // Initialisation en esclave sur l'adresse 0x71
  Wire.onReceive(wireReceiveEventFunction); // Événement réception
  Wire.onRequest(wireRequestEventFunction); // Événement émission
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // La LED signale le démarrage
  delay(500);

  byte i;
  for(i=0; i<8; i++) // Initialisation des sorties
  {
    pinMode(eaRows[i],OUTPUT); // Toutes les lignes de la matrice en sortie
    digitalWrite(eaRows[i],LOW);
    pinMode(eaCols[i],OUTPUT);
    digitalWrite(eaCols[i],LOW);
  }

  byte* tablePtr = (byte*)progTable; // Pointeur sur la table
  for(i=0; i<sizeof(progTable); i++)
  {
    *tablePtr++=EEPROM.read(i); // Copie directe via le pointeur
  }
  stackIn=stackOut=0;

  for(i=0; i<NUM_AIG; i++)  // Initialise tous les aiguillages
    setPos(i,0);

  digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
  if(stackIn!=stackOut) // Il y a un aiguillage à manoeuvrer
    eaSwitch(aigStack[stackOut++]);
  else
    delay(100); // Rien à faire
}

void wireReceiveEventFunction() // Fonction de réception
{
  while(!Wire.available()); // Attend les données
  cmd = Wire.read(); // Lecture de la commande
  dataSize = (cmd==0x34)?3:1;   // La taille des données dépend de la commande
  byte i=0;
  while(Wire.available()) // Tant que des données sont disponibles
  {
    data[i++]=Wire.read(); // Lecture des données
    if(i==dataSize) break;
  }
  switch(cmd)
  {
    case 0x30 :
      setPos(data[0],0);
      break;
    case 0x31 :
      setPos(data[0],1);
      break;
    case 0x32 :
      setPos(data[0]>>1,data[0]&1);
      break;
    case 0x33 :
      break;
    case 0x34 :
      setProg(data[0],data[1],data[2]);
      break;
    case 0x35 :
      break;
    case 0x36 :
      directSwitch(data[0]);
      break;
    default :
      cmd = 0; // commande inconnue
  }
} 

void wireRequestEventFunction() // Fonction d'émission
{
  switch(cmd)
  {
    case 0x33 : // retourne la position
      Wire.write(getPos(data[0]));
      break;
    case 0x35 : // retourne la configuration
      Wire.write(getProg(data[0],0));
      Wire.write(getProg(data[0],1));
      break;
  }
}

