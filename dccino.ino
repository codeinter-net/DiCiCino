// --------------------------------------------------------------------
// --- DCC : Centrale de commande de trains au format DCC           ---
// --- Pascal Barlier 08/10/2014                                    ---
// --------------------------------------------------------------------

#include <Wire.h>
#include <LiquidCrystal_I2C.h>  // http://playground.arduino.cc/Code/LCDAPI
#include <FlexiTimer2.h>        // http://playground.arduino.cc/Main/FlexiTimer2
#include <EEPROM.h>

LiquidCrystal_I2C lcd(0x20,20,4);  // Configurer le panneau LCD à l'adresse 0x20 ou changer le paramètre ici

// --------------------------------------------------------------------
// --- Paramétrage de la centrale                                   ---
// --------------------------------------------------------------------

#define DCC_HEADER_SIZE 20	 // Nombre de bits du préambule
#define DCC_PACKET_SIZE 6        // Taille maximum d'un paquet DCC
#define DCC_STACK_SIZE 64	 // Taille de la pile DCC - Attention : prend beaucoup d'espace en RAM
#define LOCO_SIZE 20             // Taille de la liste des locos - Attention : prend beaucoup d'espace en RAM

#define EEPROM_LOCOLIST 20       // Adresse de la liste des locos en mémoire non volatile (20-979)
#define EEPROM_CONTROL 0         // Adresse de la liste des contrôleurs (0-7) (+réservé 8-15)
#define EEPROM_SPEED 16          // Adresse de la constante de calcul de la vitesse moyenne (16-17)

// --------------------------------------------------------------------
// --- Paramétrage des entrées - sorties                            ---
// --------------------------------------------------------------------

#define OUTPUT_LED 13 // Diode de contrôle
#define DCC_OUT1 A5   // Sortie du signal DCC
#define DCC_OUT2 A4   // Activation du signal DCC
#define CTRL_SPEED 0  // Réglage de la vitesse
#define CTRL_F0 1     // Avant / Arrière
#define CTRL_F12 2    // F1 et F2
#define CTRL_F34 3    // F3 et F4
#define CTRL_MUXEN 12 // Activation du multiplexeur d'adresses
#define CTRL_MUXA0 4  // Adresse 0 du multiplexeur d'adresses
#define CTRL_MUXA1 5  // Adresse 1 du multiplexeur d'adresses
#define CTRL_MUXA2 6  // Adresse 2 du multiplexeur d'adresses

#define KBD_COL1 4    // Colonnes du clavier
#define KBD_COL2 5
#define KBD_COL3 6
#define KBD_COL4 7
#define KBD_ROW1 8    // Rangées du clavier
#define KBD_ROW2 9
#define KBD_ROW3 10
#define KBD_ROW4 11

// D0 réservé RX USB
// D1 réservé TX USB
// D2 réservé SDA I2C
// D3 réservé SCL I2C

// --------------------------------------------------------------------
// --- Interface DCC                                                ---
// --------------------------------------------------------------------

// Machine à états DccByteMode : séquence l'envoi des octets de données DCC

#define DCC_PACKET_IDLE 0
#define DCC_PACKET_HEADER 1
#define DCC_PACKET_START 2
#define DCC_PACKET_BYTE 3
#define DCC_PACKET_STOP 4

// Machine à états DccBitMode : séquence l'envoi des bits

#define DCC_BIT_HIGH 0
#define DCC_BIT_HIGH0 1
#define DCC_BIT_LOW 2
#define DCC_BIT_LOW0 3

typedef struct _dcc_stack_
{
  byte	loco;
  byte	type;
  byte	prio;
  byte	rep;       // Répétition : 0  permanent
  byte	data[DCC_PACKET_SIZE];  // Données DCC à envoyer
  byte  size;      // Longueur du paquet
  byte  z;         // Filler pour passer la structure à 12 octets -> TODO gagner sur data ou sur type/prio pour descendre à 10 octets, voire même à 8 octets en regroupant type/prio/rep/size
} DCC_STACK;

DCC_STACK DccStack[DCC_STACK_SIZE];
volatile byte DccStackUsed=0;

byte DccPacketCount;  // Index du paquet en cours d'envoi
byte DccByteCount;    // Index de l'octet en cours d'envoi dans le paquet
byte DccByteMode;     // Machine à états : préambule / start / donnée / stop
byte DccBitCount;     // Comptage des bits de l'octet à envoyer en suivant la séquence : 0x80 0x40 0x20 0x10 0x8 0x4 0x2 0x1
byte DccBitMode;      // Machine à états : fabrication des bits par modulation de la sortie
byte DccBit;          // Mémorisation du bit en cours d'envoi

void DccStackDebug(void)
// -- Sortie du contenu de la pile DCC sur le port série à fin de débuggage --
{
  Serial.print("[");
  Serial.print(DccStackUsed);
  Serial.println("]");
  int i,j;
  for(i=0; i<DccStackUsed; i++)
  {
    Serial.print(i);
    DCC_STACK* DccPacket=&DccStack[i];
    Serial.print(" (");
    Serial.print(DccPacket->size);
    Serial.print(") :");
    for(j=0; j<DccPacket->size; j++)
    {
      Serial.print(" ");
      Serial.print(DccPacket->data[j],HEX);
    }
    Serial.println();
  }
  Serial.println();
}

void DccInt(void) 
// -- Emission des données DCC sous interruption --
{
static byte DccHeaderCount;	// Comptage des bits à un du préambule

  static word TestCount = 0;
  digitalWrite(OUTPUT_LED,!((++TestCount)&0x2000));  // Signal clignotant pour indiquer l'émission des données DCC

  switch(DccBitMode)
  {
  case DCC_BIT_HIGH :
    switch(DccByteMode)
    {
    case DCC_PACKET_IDLE :
      if(DccStackUsed)
      {
      	DccPacketCount=0;
      	DccByteMode=DCC_PACKET_HEADER;
      	DccHeaderCount=DCC_HEADER_SIZE;
      }
      break;
    case DCC_PACKET_HEADER :
      DccBit=1;
      if(!--DccHeaderCount)
      {
        DccByteMode=DCC_PACKET_START;
        DccByteCount=0;
      }
      break;
    case DCC_PACKET_START :
      DccBit=0;
      DccBitCount=0x80;
      DccByteMode=DCC_PACKET_BYTE;
      break;
    case DCC_PACKET_BYTE :
      DccBit=!!(DccStack[DccPacketCount].data[DccByteCount]&DccBitCount);  // Extraction d'un bit - l'opérateur !! transforme toute valeur différente de zéro en un
      DccBitCount>>=1;
      if(!DccBitCount)
      {
        if(DccStack[DccPacketCount].size==++DccByteCount)	// Fin du paquet
          DccByteMode=DCC_PACKET_STOP;
        else
          DccByteMode=DCC_PACKET_START;
      }
      break;
    case DCC_PACKET_STOP :
      if(++DccPacketCount==DccStackUsed)	// dernier paquet - TODO : Sauter les paquets vides (loco=0xFF)
        DccPacketCount=0;
      DccBit=1;
      DccByteMode=DCC_PACKET_HEADER;
      DccHeaderCount=DCC_HEADER_SIZE;
      break;
    }
    digitalWrite(DCC_OUT1,1);
    if(DccBit)
      DccBitMode=DCC_BIT_LOW;
    else
      DccBitMode=DCC_BIT_HIGH0;
    break;
  case DCC_BIT_HIGH0 :
    digitalWrite(DCC_OUT1,1);
    DccBitMode=DCC_BIT_LOW;
    break;
  case DCC_BIT_LOW :
    digitalWrite(DCC_OUT1,0);
    if(DccBit)
      DccBitMode=DCC_BIT_HIGH;
    else
      DccBitMode=DCC_BIT_LOW0;
    break;
  case DCC_BIT_LOW0 :
    digitalWrite(DCC_OUT1,0);
    DccBitMode=DCC_BIT_HIGH;
    break;
  }
}

void DccInit(void)
// -- Initialisation de l'interface DCC --
{
  pinMode(OUTPUT_LED, OUTPUT);
  pinMode(DCC_OUT1, OUTPUT);
  pinMode(DCC_OUT2, OUTPUT);
  digitalWrite(OUTPUT_LED,HIGH);
  digitalWrite(DCC_OUT1,LOW);
  digitalWrite(DCC_OUT2,HIGH);

  DccPacketCount=0;
  DccByteCount=0;
  DccByteMode=DCC_PACKET_IDLE;
  DccBitMode=DCC_BIT_HIGH;
  DccBitCount=0;
  DccBit=0;

  FlexiTimer2::set(1, 0.000056, DccInt);  // Démarre l'interface sous interruption
  FlexiTimer2::start();
}

// --------------------------------------------------------------------
// --- Liste des locomotives                                        ---
// --------------------------------------------------------------------

#define LM_ID_EXT   0x10  // Adressage étendu (9999 locos)
#define LM_CRANS    0xF
#define LM_CRANS14  1
#define LM_CRANS27  2
#define LM_CRANS28  3
#define LM_CRANS128 4

#define MODE_AV     3
#define MODE_AR     0
#define MODE_STOP   1

typedef struct _loco_
{
  word  id;
  char  name[8];
  byte  vmax;
  byte  mode;
} LOCO;  // 12 octets

LOCO LocoList[LOCO_SIZE];
byte LocoIndex[LOCO_SIZE];

void LocoListRead(void)
// -- Lecture de la liste des locos depuis l'EEPROM --
{
  byte i;
  byte* data=(byte*)LocoList;
  for(i=0; i<sizeof(LocoList); i++)
    data[i]=EEPROM.read(i+EEPROM_LOCOLIST);
}

void LocoWrite(byte loco)
// -- Ecriture des paramètres d'une loco dans l'EEPROM --
{
  byte i;
  byte* data=(byte*)&LocoList[loco];
  byte index=loco*sizeof(LOCO)+EEPROM_LOCOLIST;
  for(i=0; i<sizeof(LOCO); i++)
    EEPROM.write(i+index,data[i]);
}

void LocoListSort(void)
// -- Tri alphabétique de la liste des locos --
{
  byte i,j;
  byte lastIndex=0xFF;
  byte curIndex=0xFF;

  for(j=0; j<LOCO_SIZE; j++)
  {
    if((*LocoList[j].name)&&((curIndex==0xFF)||(strncmp(LocoList[j].name,LocoList[curIndex].name,8)<0)))  // TODO : problème si plus d'une loco a le même nom
      curIndex=j;
  }
  LocoIndex[0]=lastIndex=curIndex;
  for(i=1; i<LOCO_SIZE; i++)
  {
    curIndex=0xFF;
    for(j=0; j<LOCO_SIZE; j++)
    {
      if((*LocoList[j].name)&&(strncmp(LocoList[j].name,LocoList[lastIndex].name,8)>0)&&((curIndex==0xFF)||(strncmp(LocoList[j].name,LocoList[curIndex].name,8)<0)))
      curIndex=j;
    }
    LocoIndex[i]=lastIndex=curIndex;
  }
}

byte NewLoco(void)
// -- Recherche un emplacement libre dans la liste --
{
  byte  i;
  for(i=0; i<LOCO_SIZE; i++)
    if(!LocoList[i].id)
      return(i);
  return(0xFF);
}

void LocoListExport(void)
// -- Exporte la liste des locos vers le port série --
{
  byte i,mode;
  char name[9];
  name[8]=0;
  Serial.println("+LOCO");
  for(i=0; i<LOCO_SIZE; i++)
  {
    Serial.print(LocoList[i].id);
    Serial.print(",");
    memcpy(name,LocoList[i].name,8);
    Serial.print(name);
    Serial.print(",");
    Serial.print(LocoList[i].vmax*10);
    Serial.print(",");
    Serial.print(LocoList[i].mode&LM_ID_EXT?"9999":"99");
    Serial.print(",");
    switch(LocoList[i].mode&LM_CRANS)
    {
      case LM_CRANS14  : mode=14; break;
      case LM_CRANS27  : mode=27; break;
      case LM_CRANS28  : mode=28; break;
      case LM_CRANS128 : mode=128; break;
    }
    Serial.println(mode);
  }
  Serial.println("-LOCO");
}

// --------------------------------------------------------------------
// Gestion de la pile DCC
// --------------------------------------------------------------------

enum packet_type{DCC_SPEED=1,DCC_FCT,DCC_CV};
enum packet_prio{PP_URGENT=1,PP_NORMAL,PP_SLOW,PP_URGENT_NORMAL,PP_NORMAL_SLOW};

// Dans la pratique :
// - On utilise la priorité urgent pour les variables de configuration
// - La priorité normal sert pour la vitesse
// - La priorité lent sert pour les fonctions
// - On peut utiliser urgent puis normal pour les changements de vitesse
// - On peut utiliser normal puis lent pour les changement d'état des fonctions

struct _dcc_stack_* StackSearch(byte loco,byte type)
// -- Recherche un paquet dans la pile --
// Si la pile contient déjà ce même paquet -> retourne son emplacement pour réutilisation
// Sinon recherche le premier emplacement libre dans la pile
// Retourne zéro si la pile est pleine
{
  DCC_STACK* DccPtr=DccStack;
  char i;
//Serial.println("SEARCH");
  for(i=DccStackUsed; --i>=0;)
  {
    if((DccPtr->loco==loco)&&(DccPtr->type==type))
      return(DccPtr);
    DccPtr++;
  }
  DccPtr=DccStack;
  for(i=DccStackUsed; --i>=0;)
  {
    if(DccPtr->loco==0xFF)  // Emplacement libre
    {
      DccStackUsed++;
      return(DccPtr);
    }
    DccPtr++;
  }
  if(DccStackUsed<DCC_STACK_SIZE)  // La pile n'est pas pleine
  {
    DccStackUsed++;
    return(DccPtr);
  }

  return(0);
}

void StackAdd(byte loco, byte type, byte prio, byte rep, byte* data, byte size)
// -- Ajoute un paquet DCC à la pile --
// loco : numéro de loco
// type : Type d'ordre : vitesse / fonction / configuration
// prio : Priorité : urgent / normal / lent / urgent une fois puis normal / normal une fois puis lent
// rep  : Répétition : permanent / n fois
// data : Données à stocker
// size : taille des données
{
  if(!size||(size>6)) return;
  DCC_STACK* DccPacket=StackSearch(loco,type);
  if(!DccPacket) return;

  DccPacket->size=0;  // Stoppe l'émission du paquet pendant la copie
  DccPacket->loco=loco;
  DccPacket->type=type;
  DccPacket->prio=prio;
  DccPacket->rep=rep;
  memcpy(DccPacket->data,data,size);
  DccPacket->size=size;
}

void StackDel(byte loco,byte type)
// -- Retire de la pile un ou tous les paquets correspondant à une loco --
// loco : locomotive concernée
// type : type de paquet à supprimer ; si = 0 -> tous les paquets sont supprimés quelqu'en soit le type
{
  DCC_STACK* DccPtr=DccStack;
  char i;
  for(i=DccStackUsed; --i>=0;)
  {
    if((DccPtr->loco==loco)&&(!type||(DccPtr->type==type)))
      DccPtr->loco=0xFF;
    DccPtr++;
  }
}

byte StackGet(byte loco, byte type, byte* data, byte size)
// -- Lit le paquet correspondant à une commande stockée --
// loco : numéro de loco
// type : Type d'ordre : vitesse / fonction / configuration
// data : Données à lire
// size : Taille du buffer
// -> Taille lue
// TODO : Choisir le numéro de registre dans le cas de DCC_FCT -> Ce n'est pas forcément idéal comme méthode
{
  DCC_STACK* DccPtr=DccStack;
  char i;
  for(i=DccStackUsed; --i>=0;)
  {
    if((DccPtr->loco==loco)&&(DccPtr->type==type)&&DccPtr->size)
    {
      if(DccPtr->size<size) size=DccPtr->size;
      memcpy(data,DccPtr->data,size);
      return(DccPtr->size);
    }
    DccPtr++;
  }
  return(0);
}

// --------------------------------------------------------------------
// Formatage des paquets DCC
// --------------------------------------------------------------------

void DCCFormat(byte Type, byte Num, word Data1, word Data2)
// -- Formate les données au format DCC et les stocke dans la pile --
// Type : Type d'ordre : vitesse / fonction / configuration
// Num  : Numéro de la locomotive
// Data1/Data2 : Données - dépend du type d'ordre
{
  byte	Size=0;
  byte	Cran;
  byte  Checksum=0;

  struct _loco_ *Loco;
  Loco=LocoList+Num;

  byte DccDataBuffer[6];
  byte* DccData=DccDataBuffer;

  word Id=Loco->id;
  byte Mode=Loco->mode;

  if(Mode&LM_ID_EXT)	// Identifiant de la loco
  {
    Checksum^=*DccData++=(Id>>8)|0xC0;
    Checksum^=*DccData++=Id&0xFF;
    Size+=2;
  }
  else
  {
    Checksum^=*DccData++=Id&0x7F;
    Size++;
  }

  switch(Type)
  {
  case DCC_SPEED :  // Data1 = vitesse ; Data2 = AV / AR / STOP
    if(Data2==MODE_STOP) Data1=0;
  
    switch(Mode&LM_CRANS)	// Consigne de vitesse
    {
    case LM_CRANS28 :
      Cran=((byte)(((long)Data1*29)>>10))+3;
      if(Cran<4) Cran=0;	// Arrêt
      Checksum^=*DccData++=0x40|((Data2==MODE_AR)?0:0x20)|(Cran>>1)|((Cran&1)<<4);
      Size++;
      break;
    case LM_CRANS128 :
      Cran=Data1>>3;
      if(Cran==1) Cran=0; // Le cran 1 correspond au freinage d'urgence
      Checksum^=*DccData++=0x3F;
      Checksum^=*DccData++=((Data2==MODE_AR)?0:0x80)|Cran;
      Size+=2;
      break;
    }    break;
  case DCC_FCT :  // Data1 = Numéro du registre ; Data = Masque des bits - TODO : devrait devenir : Data1 = Numéro de la fonction ; Data2= On/Off
    switch(Data1)
    {
    case 0 :
      Checksum^=*DccData++=0x80|Data2;
      Size++;
    }

    break;
  }
  *DccData++=Checksum;
  Size++;
  if(Size>6) return;
  StackAdd(Num, Type, PP_NORMAL, 0, DccDataBuffer, Size);  // Stockage du paquet dans la pile
}

// --------------------------------------------------------------------
// Gestion des contrôleurs
// --------------------------------------------------------------------

typedef struct _command_
{
  byte loco;
  byte F0;
  word Speed;
  long Fct;  // Masque de bits représentant les fonctions 0 à 31
} CONTROL;

CONTROL ControlList[8]; // Liste des contrôleurs
byte ControlSelect=0;   // Contrôleur actif pour l'affichage

void ControlInit(void)
// -- Initialise les lignes d'entrée-sortie utilisées par les contrôleurs --
{
  pinMode(CTRL_MUXEN,OUTPUT);
  int i;
  for(i=0; i<sizeof(ControlList)/sizeof(CONTROL); i++)  // Tous les contrôleurs sont non affectés par défaut
  {
    ControlList[i].loco=0xFF;
    ControlList[i].F0=MODE_STOP;
  }
}

void ControlChange(char key)
// -- Change le contrôleur actif dans l'affichage --
{
  if((key<'1')||(key>'8'))
    return;
  ControlSelect=(key&0xF)-1;
}

byte ControlScan(void)
// -- Parcourt les contrôleurs pour lire les consignes appliquées aux locomotives --
// Met à jour les paquets DCC si les valeurs ont changé
// Retourne true si l'affichage doit être mis à jour
{
  byte  i;
  byte  update=false;
  CONTROL* Control=ControlList;
  for(i=0; i<8; i++)
  {
    byte loco=Control->loco;
    if(loco!=0xFF)
    {
      digitalWrite(CTRL_MUXA0,i&1);    // Sélection du contrôleur à lire
      digitalWrite(CTRL_MUXA1,!!(i&2));
      digitalWrite(CTRL_MUXA2,!!(i&4));
      digitalWrite(CTRL_MUXEN,HIGH);

      word Speed=analogRead(CTRL_SPEED);
      byte F0=analogRead(CTRL_F0)>>8;  // AR=0 - STOP=1 - AV=3

      if((Control->Speed!=Speed)||(Control->F0!=F0))  // Les paramètres ont été modifiés
      {
        Control->Speed=Speed;
        Control->F0=F0;
        DCCFormat(DCC_SPEED,loco,Speed,F0);
        DCCFormat(DCC_FCT,loco,0,(F0!=MODE_STOP)?0x10:0);  // Allumage des feux
        if(i==ControlSelect) update=true;
      }
      digitalWrite(CTRL_MUXEN,LOW);
    }
    Control++;
  }
  return(update);
}

// --------------------------------------------------------------------
// Lecture du clavier
// --------------------------------------------------------------------

const byte KbdCol[4]={KBD_COL1,KBD_COL2,KBD_COL3,KBD_COL4};
const byte KbdRow[4]={KBD_ROW1,KBD_ROW2,KBD_ROW3,KBD_ROW4};
const byte KbdOut[4][4]={{'1','4','7','*'},{'2','5','8','0'},{'3','6','9','#'},{'A','B','C','D'}};

void KeyboardInit(void)
// -- Initialise les lignes d'entrée-sortie utilisées par le clavier --
{
  int row,col;
  for(col=0; col<4; col++)
    pinMode(KbdCol[col],OUTPUT);
  for(row=0; row<4; row++)
    pinMode(KbdRow[row],INPUT_PULLUP);
}

char KeyboardScan(void)
// -- Parcourt la matrice du clavier à la recherche d'une touche pressée --
{
  int row,col,col2;
  for(col=0; col<4; col++)
  {
    for(col2=0; col2<4; col2++)
      digitalWrite(KbdCol[col2],HIGH);
    digitalWrite(KbdCol[col],LOW);
    for(row=0; row<4; row++)
      if(!digitalRead(KbdRow[row])) return(KbdOut[col][row]);
  }
  return(' ');
}

char KeyboardRead()
// -- Lecture du clavier --
{
  static char lastKey=0;
  char newKey=KeyboardScan();
  if(newKey!=lastKey)
  {
    delay(50);  // Anti rebond
    lastKey=newKey;
    return(newKey);
  }
  return(0);
}

// --------------------------------------------------------------------
// Saisie de texte ou de valeurs numériques
// --------------------------------------------------------------------

#define INPUT_INIT 0xFFFF
#define INPUT_ESC 0xFFFE
#define INPUT_CONT 0xFFFD

const char TxtAlphaList[10][4]=
{
  {' ','=',':','#'},{'+','-','*','/'},{'A','B','C',0},{'D','E','F',0},{'G','H','I',0},
  {'J','K','L',0},{'M','N','O',0},{'P','Q','R','S'},{'T','U','V',0},{'W','X','Y','Z'}
};

char* InputNum(char key, byte x, byte y, byte size,char* init)
// -- Saisie d'une valeur numérique --
// Key  : Touche pressée
// x/y  : position d'affichage
// size : Nombre maximum de digits
// init : Valeur initiale
{
  static byte pos;
  static char data[10];

  if(key==0)	// Initialisation
  {
    if(init)
    {
      strncpy(data,init,size);
      pos=strlen(data);
    }
    else
    {
      *data=0;
      pos=0;
    }
    lcd.setCursor(x+pos,y);
    lcd.blink();
    return((char*)INPUT_INIT);
  }
  else if(key=='#')	// OK
  {
    lcd.noBlink();
    data[pos]=0;
    return(data);
  }
  else if(key=='*')	// ESC
  {
    lcd.noBlink();
    return((char*)INPUT_ESC);	// code retour ESC
  }
  else if((key=='A')&&(pos>0))	// effacement
  {
    pos--;
    data[pos]=0;
    lcd.setCursor(x+pos,y);
    lcd.write(' ');
    lcd.setCursor(x+pos,y);
  }
  else if((key>='0')&&(key<='9')&&(pos<size))	// Appui sur un chiffre
  {
    lcd.setCursor(x+pos,y);
    lcd.write(key);
    data[pos++]=key;
  }
  return((char*)INPUT_CONT);	// code retour signifiant que la saisie n'est pas terminée
}

char* InputAlpha(char key, byte x, byte y, byte size,char* init)
// -- Saisie d'une valeur texte - fonctionnement identique à un clavier SMS --
// Key  : Touche pressée
// x/y  : position d'affichage
// size : Nombre maximum de caractères
// init : Valeur initiale
{
  static byte pos;
  static char data[10];
  static long lastMilis=0;
  static char lastKey;

  if(key==0)	// Initialisation
  {
    if(init)
    {
      strncpy(data,init,size);
      pos=strlen(data);
    }
    else
    {
      *data=0;
      pos=0;
    }
    lcd.setCursor(x+pos,y);
    lcd.blink();
    lastMilis=millis();
    lastKey=0;
    return((char*)INPUT_INIT);
  }
  else if(key=='#')	// OK
  {
    lcd.noBlink();
    data[pos]=0;
    return(data);
  }
  else if(key=='*')	// ESC
  {
    lcd.noBlink();
    return((char*)INPUT_ESC);	// code retour ESC
  }
  else if((key=='A')&&(pos>0))	// effacement
  {
    pos--;
    data[pos]=0;
    lcd.setCursor(x+pos,y);
    lcd.write(' ');
    lcd.setCursor(x+pos,y);
    lastKey=0;
  }
  else if((key>='0')&&(key<='9'))	// Appui sur un chiffre
  {
    char outChar=key;
    if((pos==size)||((key==lastKey)&&(millis()-lastMilis<1000)&&(pos>0)))	// Même touche à une seconde d'intervalle -> Accès aux caractères alphabétiques
    {
      char lastChar=data[--pos];	// On récupère le caractère précédent
      char* TxtAlphaSubList=(char*)&TxtAlphaList[key&0xF];	// Série de caractères affectée à la touche
      if((lastChar>='0')&&(lastChar<='9'))	// Si un chiffre est déjà présent, on prend directement le premier caractère de la série
      {
        outChar=*TxtAlphaSubList;
      }
      else	// Si un caractère est déjà présent, il faut rechercher le caractère suivant ou revenir au chiffre d'origine
      {
        byte i;
        for(i=0; i<3; i++)	// Le chiffre d'origine est déjà présent dans la variable key -> Il n'y a gérer que les changements de caractères alphabétiques
        {
          if(TxtAlphaSubList[i]&&(TxtAlphaSubList[i]==lastChar)&&TxtAlphaSubList[i+1])
            outChar=TxtAlphaSubList[i+1];
        }
      }
    }
    lcd.setCursor(x+pos,y);
    lcd.write(outChar);
    data[pos++]=outChar;
    lastMilis=millis();
    lastKey=key;
  }
  return((char*)INPUT_CONT);	// code retour signifiant que la saisie n'est pas terminée
}

// --------------------------------------------------------------------
// Affichage de la consigne de vitesse avec des grands chiffres
// --------------------------------------------------------------------

// Définition des 8 caractères programmables pour composer les grands chiffres

byte ProgChar[8][8]=
{
  {B00001,B00111,B01111,B01111,B11111,B11111,B11111,B11111},
  {B10000,B11100,B11110,B11110,B11111,B11111,B11111,B11111},
  {B11111,B11111,B11111,B11111,B01111,B01111,B00111,B00001},
  {B11111,B11111,B11111,B11111,B11110,B11110,B11100,B10000},
  {B00001,B00011,B00011,B00111,B00111,B01111,B01111,B11111},
  {B11111,B11110,B11110,B11100,B11100,B11000,B11000,B10000},
  {B11111,B11111,B11111,B11111,B00000,B00000,B00000,B00000},
  {B00000,B00000,B00000,B00000,B11111,B11111,B11111,B11111}
};

// Création des chiffres de 0 à 9 à partir des caractères programmables + espace (32) et pavé plein (255)

const byte BigDigit[10][4][4]=
{
  {{0,6,1,32},{255,32,255,32},{255,32,255,32},{2,7,3,32}},
  {{4,255,32,32},{32,255,32,32},{32,255,32,32},{7,255,7,32}},
  {{0,6,1,32},{32,4,5,32},{4,5,32,32},{255,7,7,32}},
  {{0,6,1,32},{32,7,3,32},{32,6,1,32},{2,7,3,32}},
  {{32,4,255,32},{4,5,255,32},{255,7,255,32},{32,32,255,32}},
  {{255,6,6,32},{6,6,1,32},{32,32,255,32},{2,7,3,32}},
  {{0,6,6,32},{255,6,1,32},{255,32,255,32},{2,7,3,32}},
  {{6,6,255,32},{32,4,5,32},{4,5,32,32},{255,32,32,32}},
  {{0,6,1,32},{2,7,3,32},{0,6,1,32},{2,7,3,32}},
  {{0,6,1,32},{2,7,255,32},{32,32,255,32},{2,7,3,32}}
};

void BigDigitInit(void)
// -- Initialisation des caractères programmables --
{
  int i;
  for(i=0; i<8; i++)
    lcd.createChar(i,ProgChar[i]);
}

void BigDigitDisplay(word Num,byte Pos)
// -- Affiche un chiffre en gros caractères sur 4 lignes--
// Num : Nombre à afficher compris entre 0 et 999
// Pos : Position où commence l'affichage
{
  byte digit[3];
  digit[2]=Num%10;
  Num/=10;
  digit[1]=Num%10;
  Num/=10;
  digit[0]=Num%10;

  for(int y=0; y<4; y++)
  {
    lcd.setCursor(Pos,y);
    for(int x=0; x<3; x++)
    {
      byte d=digit[x];
      for(int xx=0; xx<((x==2)?3:4); xx++)
        lcd.write(BigDigit[d][y][xx]);
    }
  }
}

// --------------------------------------------------------------------
// Affichages contextuels
// --------------------------------------------------------------------

#define DLI_DIRECTION 1  // Affichage du sens de marche
#define DLI_1LINE 0  // Affichage sur une ligne
#define DLI_2LINES 2  // Affichage sur deux lignes

void DisplayText(char* Name,byte Size)
// -- Affiche un texte et complète avec des blancs jusqu'au nombre de caractères indiqués --
{
//  Serial.println(Name);
  return;

  char j;
  for(j=Size;--j>=0;)
    lcd.write(*Name?*Name++:' ');
}

void DisplayLocoInfo(byte Mode)
// -- Affiche le numéro de la loco, son nom et son sens --
// Mode&DLI_DIRECTION : Affiche le sens de marche
// Mode&DLI_2LINES : Affichage sur deux lignes
{
  lcd.setCursor(0,0);
  lcd.print(ControlSelect+1);
  if(Mode&DLI_DIRECTION)
  {
    switch(ControlList[ControlSelect].F0)
    {
      case MODE_AV : lcd.write(126); break;  // Caractère "->"
      case MODE_AR : lcd.write(127); break;  // Caractère "<-"
      default : lcd.print("-");
    }
  }
  else
    lcd.write(' ');
  if(Mode&DLI_2LINES) lcd.setCursor(0,1);  // Affichage sur deux lignes
  byte loco=ControlList[ControlSelect].loco;
  if(loco!=0xFF)
    DisplayText(LocoList[loco].name,8);
  else
    lcd.print("--------");
  // TODO : afficher la vitesse mesurée par le rail de détection
}

void DisplayLocoSpeed()
// -- Affiche la consigne de vitesse en gros chiffres et avec un bargraphe --
{
  byte loco=ControlList[ControlSelect].loco;
  word vmax=(loco!=0xFF)?LocoList[loco].vmax*10:0;
  long Speed=ControlList[ControlSelect].Speed;

  BigDigitDisplay((word)((Speed*(vmax+1))>>10),9);
  Speed=(Speed+14)>>7;
  lcd.setCursor(0,3);
  int i;
  for(i=0;i<8;i++)
    lcd.write((i<Speed)?219:95);  // Bargraphe de puissance moteur
}

byte DisplayLocoList(char Change)
// -- Affichage de la liste des locos par groupes de 4 avec une flèche de sélection --
// Change : si différent de zéro -> déplacement dans la liste
{
  static byte Pos=0;
  if(Change<0)  // Remonte
  {
    if(Pos==0)  // Début de liste
      return(LocoIndex[Pos]);
  }
  else if(Change>0)  // Descend
  {
    if((Pos+1==(sizeof(LocoList)/sizeof(LOCO)))||(LocoIndex[Pos+1]==0xFF)) // Fin de liste
      return(LocoIndex[Pos]);
  }
  Pos+=Change;

  byte i;
  for(i=0;i<4;i++)  // Affiche 4 locos
  {
    if((Pos&0xFC)+i==sizeof(LocoList)/sizeof(LOCO)) break; // Fin de liste
    lcd.setCursor(0,i);
    lcd.write(((Pos&3)==i)?126:32);  // Affiche une flèche de sélection
    if(LocoIndex[(Pos&0xFC)+i]!=0xFF)
      DisplayText(LocoList[LocoIndex[(Pos&0xFC)+i]].name,8);
    else
      lcd.print("        ");
  }
  return(LocoIndex[Pos]);
}

void DisplayControl(void)
// -- Affichage de la liste des locos rattachées aux controleurs --
{
  lcd.clear();
  int i;
  byte loco;
  for(i=0 ; i<4; i++)
  {
    lcd.setCursor(0,i);
    lcd.print(i+1);
    lcd.print(" ");
    loco=ControlList[i].loco;
    if(loco!=0xFF)
      DisplayText(LocoList[loco].name,8);
    else
      lcd.print("--------");
    lcd.print(i+5);
    lcd.print(" ");
    loco=ControlList[i+4].loco;
    if(loco!=0xFF)
      DisplayText(LocoList[loco].name,8);
    else
      lcd.print("--------");

  }
}

void DisplayLocoParam(char* Name, char* Id, char* Vmax, word CrAd)
// -- Affiche les paramètres de contrôle de la locomotive --
// Si un paramètre est à zéro -> pas d'affichage de ce paramètre
{
  if(Name)
  {
    lcd.setCursor(7,0);
    DisplayText(Name,8);
  }
  if(Id)
  {
    lcd.setCursor(6,1);
    DisplayText(Id,4);
  }
  if(Vmax)
  {
    lcd.setCursor(6,2);
    DisplayText(Vmax,3);
  }
  if(CrAd)
  {
    lcd.setCursor(6,3);
    switch(CrAd&LM_CRANS)
    {
      case LM_CRANS14 : lcd.print("14 "); break;
      case LM_CRANS27 : lcd.print("27 "); break;
      case LM_CRANS28 : lcd.print("28 "); break;
      case LM_CRANS128 : lcd.print("128"); break;
    }
    lcd.setCursor(14,3);
    lcd.print(CrAd&LM_ID_EXT?"9999":"99  ");
  }
}

// --------------------------------------------------------------------
// Interface utilisateur
// --------------------------------------------------------------------

#define UIM_NORMAL 0
#define UIM_EDIT 1
#define UIE_KEY_SPEED 0xFF
#define UIE_INIT 1
#define UIE_EDIT 2
#define UIE_EXIT 3
#define UIE_DISP 4

typedef struct _display_list_
{
  byte x;
  byte y;
  const char* txt;
} DISPLAY_LIST;

typedef struct _ui_action_
{
  byte in;
  byte go;
} UI_ACTION;

typedef struct _ui_page_
{
  const DISPLAY_LIST *text;
  byte textSize;
  const UI_ACTION *action;
  byte actionSize;
} UI_PAGE;

// Tous les textes affichés par l'interface utilisateur
// Ils sont regroupés car ils peuvent être utilisés plusieurs fois

const char TxtStart1[]={255,255,255,32,32,32,255,255,255,32,32,255,255,255,0};
const char TxtStart2[]={255,32,32,255,32,255,32,32,32,32,255,0};
const char TxtMenu[]="[#]menu";
const char TxtOk[]="[#]ok";
const char TxtOkEsc[]="[#]/[*]";
const char TxtFct[]={'1',7,'2',7,'3',7,'4',7,0};
const char TxtSelect[]="[#]select";
const char TxtEsc[]="[*]esc";
const char TxtConfig[]="[0]config";
const char TxtParam[]="[A]param";
const char TxtFonctions[]="[B]supp.";
const char TxtCV[]="[C]cv";
const char TxtNouveau[]="[D]nouveau";
const char TxtHaut[]="[C]haut";
const char TxtBas[]="[D]bas";
const char TxtNom[]="[A]nom=";
const char TxtId[]="[B]id=";
const char TxtVm[]="[C]vm=";
const char TxtCr[]="[D]cr=";
const char TxtAd[]="ad=";
const char TxtConfigLoop[]="[A]circuit=";
const char TxtGauche[]="[A]<-";
const char TxtDroit[]="[B]->";
const char TxtAlpha1[]="1+-*/ 2ABC 3DEF #ok";
const char TxtAlpha2[]="4GHI  5JKL 6MNO *esc";
const char TxtAlpha3[]="7PQRS 8TUV 9WXYZ 0=:";
//const char TxtCv[]="[C]cv";
//const char TxtD[]="[D]";
//const char TxtB[]="[B]";
const char Txt[]="[]";
const char TxtCvAddress[]="[A] adresse";
const char TxtCvBroadcast[]="[B] broadcast";
const char TxtCvSpeed[]="[B] vitesse";
const char TxtCvOther[]="[C] autres";
const char TxtCvAll[]="[D] toutes";
const char TxtCvAdressInput[]="Adresse=";
const char TxtCvSpeedList1[]="1 Vmin 2 Vmax 3 Vmid";
const char TxtCvSpeedList2[]="4 Tacc 5 Tdec 6 Pmid";
const char TxtCvSpeedList3[]="7 Vsta";
const char TxtCvOtherList1[]="1 Seuil regul";
const char TxtCvOtherList2[]="2 Duree memo";
const char TxtCvAllTitle[]="CV individuelle";
const char TxtCvAllNumInput[]="Numero =";
const char TxtCvAllValInput[]="Valeur =";
const char TxtCv2[]="Vitesse mini (cv2)";
const char TxtCv5[]="Vitesse maxi (cv5)";
const char TxtCv6[]="Vitesse median (cv6)";
const char TxtCv3[]="Taux accel (cv3)";
const char TxtCv4[]="Taux decel (cv4)";
const char TxtCv25[]="Pos median (cv25)";
const char TxtCv65[]="Vitesse start (cv65)";
const char TxtCv10[]="Seuil regul (cv10)";
const char TxtCv11[]="Duree memo (cv11)";

const DISPLAY_LIST DisplayStart[]={{3,0,TxtStart1},{3,1,TxtStart2},{3,2,TxtStart2},{3,3,TxtStart1}};
const DISPLAY_LIST DisplayPilot[]={{0,2,TxtFct}}; // {0,2,TxtOkEsc}
const DISPLAY_LIST DisplayMenu[]={{10,0,TxtConfig},{0,1,TxtSelect},{10,1,TxtEsc},{0,2,TxtParam},{10,2,TxtFonctions},{0,3,TxtCV},{10,3,TxtNouveau}};
const DISPLAY_LIST DisplaySelect[]={{12,0,TxtOk},{12,1,TxtEsc},{12,2,TxtHaut},{12,3,TxtBas}};
const DISPLAY_LIST DisplayParam[]={{0,0,TxtNom},{0,1,TxtId},{11,1,TxtOk},{0,2,TxtVm},{11,2,TxtEsc},{0,3,TxtCr},{11,3,TxtAd}};
const DISPLAY_LIST DisplayParamName[]={{0,0,TxtNom},{0,1,TxtAlpha1},{0,2,TxtAlpha2},{0,3,TxtAlpha3}};
//const DISPLAY_LIST DisplayCv[]={{12,0,TxtCv},{8,3,TxtEsc}};
//const DISPLAY_LIST DisplayCvData[]={{12,0,TxtCv},{0,1,TxtD},{8,1,TxtB},{0,2,TxtGauche},{0,3,TxtOk},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayConfig[]={{0,0,TxtConfigLoop},{0,3,TxtOk},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayCv[]={{0,0,TxtCvAddress},{0,1,TxtCvBroadcast},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayCvMenu[]={{0,0,TxtCvAddress},{0,1,TxtCvSpeed},{0,2,TxtCvOther},{0,3,TxtCvAll},{10,3,TxtEsc}};
const DISPLAY_LIST DisplayCvAddress[]={{0,0,TxtCvAdressInput},{0,3,TxtOk},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayCvSpeed[]={{0,0,TxtCvSpeedList1},{0,1,TxtCvSpeedList2},{0,2,TxtCvSpeedList3},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayCvOther[]={{0,0,TxtCvOtherList1},{0,1,TxtCvOtherList2},{8,3,TxtEsc}};
const DISPLAY_LIST DisplayCvAll[]={{0,0,TxtCvAllTitle},{0,0,TxtCvAllNumInput},{0,0,TxtCvAllValInput},{0,3,TxtOk},{8,3,TxtEsc}};
//const DISPLAY_LIST Display[]={{0,0,Txt},{0,3,TxtOk},{8,3,TxtEsc}};

enum UserPosition{UI_MAIN=0,UI_PILOT,UI_MENU,UI_SELECT,UI_PARAM,UI_PARAM_NAME,UI_CV,UI_CONFIG_DATA,UI_CONFIG,
UI_INTER=100,UI_MAIN_CONTROL,UI_MAIN_EXPORT,UI_MAIN_DEBUG,UI_PILOT_CONTROL,UI_MENU_CONTROL,UI_MENU_DEL,UI_MENU_NEW,UI_SELECT_OK,UI_SELECT_H,UI_SELECT_B,
UI_PARAM_OK,/*UI_PARAM_NAME,*/UI_PARAM_ID,UI_PARAM_VM,UI_PARAM_CR_AD,
UI_NOM_NUM,UI_NOM_OK,UI_NOM_G,UI_NOM_D,/*UI_CONFIG_CV,*/UI_CONFIG_L,UI_CONFIG_BIN,UI_CONFIG_DEC,
UI_MENU_CV,UI_CV_ADDR,UI_CV_SPEED,UI_CV_OTHER,UI_CV_ALL,
UI_NULL=200};

const UI_ACTION ActionMain[]={{'@',UI_MAIN_CONTROL},{'#',UI_PILOT},{'A',UI_MAIN_EXPORT},{'D',UI_MAIN_DEBUG}};
const UI_ACTION ActionPilot[]={{'@',UI_PILOT_CONTROL},{'#',UI_MENU},{'*',UI_MAIN}};
const UI_ACTION ActionMenu[]={{'0',UI_CONFIG},{'@',UI_MENU_CONTROL},{'#',UI_SELECT},{'*',UI_PILOT},{'A',UI_PARAM},{'B',UI_MENU_DEL},{'C',UI_CV},{'D',UI_MENU_NEW}};
const UI_ACTION ActionSelect[]={{'#',UI_SELECT_OK},{'*',UI_MENU},{'C',UI_SELECT_H},{'D',UI_SELECT_B}};
const UI_ACTION ActionParam[]={{'#',UI_PARAM_OK},{'*',UI_MENU},{'A',UI_PARAM_NAME},{'B',UI_PARAM_ID},{'C',UI_PARAM_VM},{'D',UI_PARAM_CR_AD}};
const UI_ACTION ActionParamName[]={{'@',UI_NOM_NUM},{'#',UI_NOM_OK},{'*',UI_PARAM},{'A',UI_NOM_G},{'B',UI_NOM_D}};
//const UI_ACTION ActionCv[]={{'*',UI_MENU},{'C',UI_CONFIG_CV}};
//const UI_ACTION ActionCvData[]={{'*',UI_CONFIG},{'A',UI_CONFIG_L},{'B',UI_CONFIG_BIN},{'C',UI_CONFIG_CV},{'D',UI_CONFIG_DEC}};
const UI_ACTION ActionConfig[]={{'*',UI_MENU}};
const UI_ACTION ActionCv[]={{'*',UI_MENU},{'A',UI_MENU_CV},{'B',UI_MENU_CV}};
const UI_ACTION ActionMenuCv[]={{'*',UI_MENU},{'A',UI_CV_ADDR},{'B',UI_CV_SPEED},{'C',UI_CV_OTHER},{'D',UI_CV_ALL}};

const UI_PAGE UserInterfaceList[]=
{
  {0,0,ActionMain,sizeof(ActionMain)/sizeof(UI_ACTION)},
  {DisplayPilot,sizeof(DisplayPilot)/sizeof(DISPLAY_LIST),ActionPilot,sizeof(ActionPilot)/sizeof(UI_ACTION)},
  {DisplayMenu,sizeof(DisplayMenu)/sizeof(DISPLAY_LIST),ActionMenu,sizeof(ActionMenu)/sizeof(UI_ACTION)},
  {DisplaySelect,sizeof(DisplaySelect)/sizeof(DISPLAY_LIST),ActionSelect,sizeof(ActionSelect)/sizeof(UI_ACTION)},
  {DisplayParam,sizeof(DisplayParam)/sizeof(DISPLAY_LIST),ActionParam,sizeof(ActionParam)/sizeof(UI_ACTION)},
  {DisplayParamName,sizeof(DisplayParamName)/sizeof(DISPLAY_LIST),ActionParamName,sizeof(ActionParamName)/sizeof(UI_ACTION)},
//  {DisplayCv,sizeof(DisplayCv)/sizeof(DISPLAY_LIST),ActionCv,sizeof(ActionCv)/sizeof(UI_ACTION)},
//  {DisplayCvData,sizeof(DisplayCvData)/sizeof(DISPLAY_LIST),ActionCvData,sizeof(ActionCvData)/sizeof(UI_ACTION)},
  {DisplayConfig,sizeof(DisplayConfig)/sizeof(DISPLAY_LIST),ActionConfig,sizeof(ActionConfig)/sizeof(UI_ACTION)}
};

void DisplayList(const struct _display_list_ *list, char size)
// -- Affiche une série de textes à divers endroits de l'écran --
// list : liste de textes à afficher
// size : taille de la liste
{
  lcd.clear();
  while(--size>=0)
  {
    lcd.setCursor(list->x,list->y);
    lcd.print(list->txt);
    list++;
  }
}

byte CrAdList[]={LM_CRANS14,LM_CRANS27,LM_CRANS28,LM_CRANS128,LM_CRANS14|LM_ID_EXT,LM_CRANS27|LM_ID_EXT,LM_CRANS28|LM_ID_EXT,LM_CRANS128|LM_ID_EXT};

byte UserInterfaceEdit(byte uiParam,byte loco,byte mode,char key)
// -- Gestion des champs éditables --
// uiParam : Identifiant du champ éditable
// loco    : Numéro de la loco
// mode    : UIE_INIT = initialisation ; UIE_EDIT = édition ; UIE_EXIT = sortie ; UIE_DISP = affichage
// key     : Touche appuyée sur le clavier
{
  static byte uiEdit;   // Page en cours d'édition
  static byte uiField;  // Champ en cours d'édition
  static char ParamName[9];
  static char ParamId[5];
  static char ParamVmax[4];
  static word ParamCrAd;
  char* ret;

  switch(mode)
  {
  case UIE_INIT : // Initialise les champs éditables
    if(!uiParam) return(0);
    uiEdit=uiParam;
    switch(uiParam)
    {
    case UI_PARAM :
      memcpy(ParamName,LocoList[loco].name,8);
      itoa(LocoList[loco].id,ParamId,10);
      itoa(LocoList[loco].vmax*10,ParamVmax,10);
      ParamCrAd=LocoList[loco].mode;
      break;
    case UI_CONFIG :
      break;
    case UI_CONFIG_DATA :
      break;
    }
    return(0);
  case UIE_DISP :  // Affiche le contenu des champs éditables
    switch(uiParam)
    {
    case UI_PARAM :
      DisplayLocoParam(ParamName,ParamId,ParamVmax,ParamCrAd);
      break;
    case UI_PARAM_NAME :
      DisplayLocoParam(ParamName,0,0,0);
      break;
    }
    return(0);
  case UIE_EDIT :  // Edition d'un champ
    if(!key)  // Prépare l'édition
    {
      if(!uiParam) return(0);
      uiField=uiParam;
    }
    if(!uiEdit||!uiField) return(0);
    switch(uiField)
    {
    case UI_PARAM_NAME :
      ret=InputAlpha(key,7,0,8,ParamName);
      break;
    case UI_PARAM_ID :
      ret=InputNum(key,6,1,4,ParamId);
      break;
    case UI_PARAM_VM :
      ret=InputNum(key,6,2,3,ParamVmax);
      break;
    case UI_PARAM_CR_AD :
      int i;
      for(i=0;i<sizeof(CrAdList);i++)  // Bouclage entre les diverses valeurs possibles pour CrAd
        if(CrAdList[i]==ParamCrAd)
          break;
      if(++i>=sizeof(CrAdList))
        ParamCrAd=CrAdList[0];
      else
        ParamCrAd=CrAdList[i];
      DisplayLocoParam(0,0,0,ParamCrAd);
      break;
    }
    break;
  case UIE_EXIT : // Stocke les champs après validation des données éditées
    switch(uiEdit)
    {
    case UI_PARAM :
      memcpy(LocoList[loco].name,ParamName,8);
      LocoList[loco].id=atoi(ParamId);
      LocoList[loco].vmax=atoi(ParamVmax)/10;
      LocoList[loco].mode=ParamCrAd;
      break;
    case UI_CONFIG :
      break;
    case UI_CONFIG_DATA :
      break;
    }
    return(0);
  }

  switch((word)ret)  // Traitement du code retour de la saisie
  {
  case INPUT_INIT :  // En cas d'initialisation -> pas de traitement
    break;
  case INPUT_ESC :  // Annuler -> sortie sans modification
    DisplayLocoParam(ParamName,ParamId,ParamVmax,ParamCrAd);
    return(UIE_EXIT);
    break;
  case INPUT_CONT :  // Continuer la saisie
    break;
  default:  // OK -> saisie validée
    switch(uiField)
    {
      case UI_PARAM_NAME :
        strncpy(ParamName,ret,sizeof(ParamName));
        break;
      case UI_PARAM_ID :
        strncpy(ParamId,ret,sizeof(ParamId));
        break;
      case UI_PARAM_VM :
        strncpy(ParamVmax,ret,sizeof(ParamVmax));
        if((ParamVmax[2]>='0')&&ParamVmax[2]<='9')  // Le chiffre des unités de Vmax est toujours égal à zéro
          ParamVmax[2]='0';
        else
          ParamVmax[1]='0';
        break;
      case UI_PARAM_CR_AD :
        break;
    }
    return(UIE_EXIT);
  }
  return(0);
}

void UserInterfaceExec(byte key)
// -- Interface utilisateur : affiche les pages et gère les touches du clavier --
// key : Touche appuyée sur le clavier
{
  static byte uiActionPos=UI_SELECT;  // Page initiale de l'interface utilisateur
  static byte uiMode=UIM_NORMAL;
  static byte loco;
  const UI_PAGE* UserInterface;

  if(key==UIE_KEY_SPEED) // Modification de la vitesse ou du sens
  {
    if((uiActionPos==UI_PILOT))
    {
      DisplayLocoInfo(DLI_2LINES|DLI_DIRECTION);
      DisplayLocoSpeed();
    }
    return;
  }

  if(uiMode==UIM_EDIT)  // Un champ éditable est actif
  {
    if(key)
    {
      byte ret=UserInterfaceEdit(0,loco,UIE_EDIT,key);
      if(ret==UIE_EXIT)
      {
        uiMode=UIM_NORMAL;
        UserInterface=&UserInterfaceList[uiActionPos];
        DisplayList(UserInterface->text,UserInterface->textSize);
        UserInterfaceEdit(uiActionPos,loco,UIE_DISP,0);
      }
    }
    return;
  }

  UserInterface=&UserInterfaceList[uiActionPos];
  byte uiActionGo=UI_NULL;  // Future page ou action manuelle

  if(!key)  // Affichage initial
  {
    uiActionGo=uiActionPos;
  }
  else  // Sélection de l'action en fonction de la touche pressée
  {
    const UI_ACTION* uiAction=UserInterface->action;
    int i;
    int testKey=key;
    if((key>='0')&&(key<='9')) testKey='@';  // Les touches de 0 à 9 sont remplacées par 0 pour le test
    for(i=UserInterface->actionSize;--i>=0;)  // Recherche si la touche pressée correspond à une action
    {
      if(uiAction->in==testKey)
      {
        uiActionGo=uiAction->go;
        break;
      }
      uiAction++;
    }
  }

  switch(uiActionGo)  // Actions spécifiques
  {
  case UI_MAIN_CONTROL :
    ControlChange(key);
    uiActionGo=UI_PILOT;
    break;
  case UI_MAIN_EXPORT :
    LocoListExport();
    break;
  case UI_MAIN_DEBUG :
    DccStackDebug();
    break;
  case UI_PILOT_CONTROL :
    ControlChange(key);
    uiActionGo=UI_PILOT;
    break;
  case UI_MENU_CONTROL :
    ControlChange(key);
    uiActionGo=UI_MENU;
    break;
  case UI_MENU_DEL :
    ControlList[ControlSelect].loco=0xFF;  // Supprime la loco de la liste
    ControlList[ControlSelect].F0=MODE_STOP;
    uiActionGo=UI_MENU;
    break;
  case UI_PARAM :
    loco=ControlList[ControlSelect].loco;  // Edition des paramètres
    if(loco!=0xFF)
      UserInterfaceEdit(UI_PARAM,loco,UIE_INIT,0);
    break;
  case UI_PARAM_OK :
    if(loco!=0xFF)
    {
      UserInterfaceEdit(0,loco,UIE_EXIT,0);
      LocoWrite(loco);
      LocoListSort();
      uiMode=UIM_NORMAL;
    }
    uiActionGo=UI_MENU;
    break;
  case UI_MENU_NEW :
    loco=NewLoco();
    if(loco!=0xFF)
    {
      LocoList[loco].mode=LM_CRANS128;
      UserInterfaceEdit(UI_PARAM,loco,UIE_INIT,0);
      uiActionGo=UI_PARAM;
    }
    break;
  case UI_SELECT_OK:
    ControlList[ControlSelect].loco=DisplayLocoList(0);  // Affecte la loco sélectionnée
    ControlList[ControlSelect].Speed=0;
    ControlList[ControlSelect].F0=0;
    ControlList[ControlSelect].Fct=0;
    uiActionGo=UI_MENU;
    break;
  case UI_SELECT_H:
    DisplayLocoList(-1);
    return;
  case UI_SELECT_B :
    DisplayLocoList(1);
    return;
  case UI_PARAM_NAME :
    UserInterface=&UserInterfaceList[UI_PARAM_NAME];
    DisplayList(UserInterface->text,UserInterface->textSize);
    UserInterfaceEdit(UI_PARAM_NAME,loco,UIE_DISP,0);
    UserInterfaceEdit(UI_PARAM_NAME,loco,UIE_EDIT,0);
    uiMode=UIM_EDIT;
    return;
  case UI_PARAM_ID :
    UserInterfaceEdit(UI_PARAM_ID,loco,UIE_EDIT,0);
    uiMode=UIM_EDIT;
    return;
  case UI_PARAM_VM :
    UserInterfaceEdit(UI_PARAM_VM,loco,UIE_EDIT,0);
    uiMode=UIM_EDIT;
    return;
  case UI_PARAM_CR_AD :
    UserInterfaceEdit(UI_PARAM_CR_AD,loco,UIE_EDIT,0);
    return;
//  case UI_CONFIG_CV :
//    uiActionGo=UI_CONFIG_DATA;
    break;
  }

  if(uiActionGo<UI_INTER)  // Changement de page
  {
    uiActionPos=uiActionGo;
    UserInterface=&UserInterfaceList[uiActionPos];
    DisplayList(UserInterface->text,UserInterface->textSize);
  }

  switch(uiActionGo)  // Changement de page
  {
  case UI_MAIN :
    DisplayControl();
    return;
  case UI_PILOT :
    DisplayLocoInfo(DLI_2LINES|DLI_DIRECTION);
    DisplayLocoSpeed();
    return;
  case UI_MENU :
    DisplayLocoInfo(DLI_1LINE);
    return;
  case UI_SELECT :
    DisplayLocoList(0);
    return;
  case UI_PARAM :
    UserInterfaceEdit(UI_PARAM,loco,UIE_DISP,0);
    return;
  case UI_CONFIG :
    DisplayLocoInfo(DLI_1LINE);
    return;
  case UI_CONFIG_DATA :
    DisplayLocoInfo(DLI_1LINE);
    return;
  }
}

// --------------------------------------------------------------------
// Initialisations et boucle principale
// --------------------------------------------------------------------

void test()
{
  ControlList[ControlSelect].loco=0;  // Affecte la première loco
  ControlList[ControlSelect].Speed=0;
  ControlList[ControlSelect].F0=0;
  ControlList[ControlSelect].Fct=0;
}

void setup()
{
  delay(1000);
  lcd.begin(20,4);
  lcd.noDisplay();
  delay(500);
  lcd.clear();
  lcd.init() ;
  DisplayList(DisplayStart,sizeof(DisplayStart)/sizeof(DISPLAY_LIST));  // Ecran de démarrage
  lcd.display();
  lcd.backlight() ;
  delay(2000);
  Serial.begin(9600);
  LocoListRead();
  LocoListSort();
  BigDigitInit();
  KeyboardInit();
  ControlInit();
  DccInit();  // Démarre l'interface DCC
  UserInterfaceExec(0);  // Initialisation de l'interface utilisateur
}

void loop()
{
  byte Key=KeyboardRead();  // Lecture du clavier
  if(Key&&(Key!=' '))
    UserInterfaceExec(Key);  // Action utilisateur

  byte displayUpdate=ControlScan();  // Lecture des contrôleurs
  if(displayUpdate)
    UserInterfaceExec(UIE_KEY_SPEED);  // Si changement de vitesse ou de sens : mise à jour de l'affichage
}

