// DiCiCino-uno
// PBA 2018-05-19
// Centrale DCC minimaliste

#include <FlexiTimer2.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define DISPLAY_WIDTH 16
#define DISPLAY_HEIGHT 2
#define LCD_BACKLIGHT 10

// ----------------------------------------
// --- Interface DCC                    ---
// ----------------------------------------

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

#define DCC_PACKET_NUM 6
#define DCC_PACKET_SIZE 6 // Taille maximum d'un paquet DCC
#define DCC_HEADER_SIZE 20
#define DCC_FUNCTION_MAX 12 // Nombre de fonctions à commander

byte DccBit; // Bit en cours d'envoi
byte DccSubBit; // Partie du bit en cours d'envoi
byte DccDataMode;    // Variable d'état de l'automate paquet
byte DccPacketUsed=0; // Nombre de paquets à envoyer
byte DccPacketIndex; // Paquet en cours d'envoi
byte DccHeaderCount; // Comptage des bits à un du préambule
byte DccByteCount;   // Index de l'octet en cours d'envoi
byte DccBitShift;    // Comptage des bits de l'octet à envoyer

byte dccPacketData[DCC_PACKET_NUM][DCC_PACKET_SIZE]; // Paquets de données à envoyer
byte dccPacketSize[DCC_PACKET_NUM]; // Taille des paquets à envoyer

void dccInterrupt(void) 
{
  switch(DccSubBit) // Automate bit
  {
  case DCC_BIT_HIGH :
    switch(DccDataMode) // Automate paquet
    {
    case DCC_PACKET_IDLE :
      if(DccPacketUsed)
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
      DccBit=!!(dccPacketData[DccPacketIndex][DccByteCount]&DccBitShift);
      DccBitShift>>=1;
      if(!DccBitShift)
      {
        if(dccPacketSize[DccPacketIndex]==++DccByteCount)  // Fin du paquet
          DccDataMode=DCC_PACKET_STOP;
        else
          DccDataMode=DCC_PACKET_START;
      }
      break;
    case DCC_PACKET_STOP :
      DccBit=1;
      if(DccPacketUsed)
      {
        for(char i=DCC_PACKET_NUM; --i>=0;)
        {
          DccPacketIndex++;
          if(DccPacketIndex==DCC_PACKET_NUM) DccPacketIndex=0;
          if(dccPacketData[DccPacketIndex][0]!=0xFF) break;
        }

        DccDataMode=DCC_PACKET_HEADER;
        DccHeaderCount=DCC_HEADER_SIZE;
      }
      else
      {
        DccDataMode=DCC_PACKET_IDLE;
      }
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

void dccAdd(byte* packetData,byte packetSize,byte index)
{
  if(packetSize>DCC_PACKET_SIZE) return;
  memcpy(dccPacketData[index],packetData,packetSize);
  dccPacketSize[index]=packetSize;
  DccPacketUsed=1;
}

void dccClear()
{
  for(int i=0; i<DCC_PACKET_NUM; i++)
  {
    dccPacketData[i][0]=0xFF;
    dccPacketSize[i]=0;
  }
  DccPacketUsed=0;
}

#define DCC_PACKET_TYPE_MODE 0xF
#define DCC_PACKET_TYPE_SPEED 0
#define DCC_PACKET_TYPE_F0_F4 1
#define DCC_PACKET_TYPE_F5_F8 2
#define DCC_PACKET_TYPE_F9_F12 3
#define DCC_PACKET_TYPE_ADDR_LONG 0x80
#define DCC_PACKET_TYPE_STEP 0x30
#define DCC_PACKET_TYPE_STEP_14 0x00
#define DCC_PACKET_TYPE_STEP_27 0x10
#define DCC_PACKET_TYPE_STEP_28 0x20
#define DCC_PACKET_TYPE_STEP_128 0x30

void dccPacketFormat(byte type, word addr, word data)
{
  byte packetData[DCC_PACKET_SIZE];
  byte checksum=0;
  byte packetSize=1;
  char* packetPtr=packetData;
  if(type&DCC_PACKET_TYPE_ADDR_LONG)
  {
    checksum^=*packetPtr++=0xC0|((addr>>8)&0x3F);
    checksum^=*packetPtr++=addr&0xFF;
    packetSize+=2;
  }
  else
  {
    checksum^=*packetPtr++=addr&0x7F;
    packetSize++;
  }
  byte dir;
  byte ext;
  switch(type&DCC_PACKET_TYPE_MODE)
  {
    case DCC_PACKET_TYPE_SPEED:
      dir=!!(data&0x100);
      switch(type&DCC_PACKET_TYPE_STEP)
      {
        case DCC_PACKET_TYPE_STEP_14:
          checksum^=*packetPtr++=(data&0xF)|(dir?0x60:0x20);
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_27:
        case DCC_PACKET_TYPE_STEP_28:
          ext=(data&1)<<4;
          data>>=1;
          checksum^=*packetPtr++=(data&0xF)|(dir?0x60:0x20)|ext;
          packetSize++;
          break;
        case DCC_PACKET_TYPE_STEP_128:
          checksum^=*packetPtr++=0x3F;
          checksum^=*packetPtr++=(data&0x7F)|(dir?0x80:0);
          packetSize+=2;
          break;
      }
      break;
    case DCC_PACKET_TYPE_F0_F4:
      checksum^=*packetPtr++=0x80|(data&0x1F);
      packetSize++;
      break;
    case DCC_PACKET_TYPE_F5_F8:
      checksum^=*packetPtr++=0xB0|(data&0xF);
      packetSize++;
      break;
    case DCC_PACKET_TYPE_F9_F12:
      checksum^=*packetPtr++=0xA0|(data&0xF);
      packetSize++;
      break;
  }
  *packetPtr=checksum;
  dccAdd(packetData,packetSize,type&DCC_PACKET_TYPE_MODE);
}

// ----------------------------------------
// --- Interface utilisateur            ---
// ----------------------------------------

#define BUTTONS_ANALOG_INPUT 0 // Entrée analogique où sont câblés les boutons
#define SPEED_ANALOG_INPUT 1

#define UI_MODE_DRAW_ALL 1
#define UI_MODE_SET_CURSOR 2
#define UI_PAGE_MAIN 1
#define UI_PAGE_PILOT 2

#define DUI_KEY_NONE   0
#define DUI_KEY_UP     1
#define DUI_KEY_DOWN   2
#define DUI_KEY_LEFT   3
#define DUI_KEY_RIGHT  4
#define DUI_KEY_ESC    5
#define DUI_KEY_OK     6
#define DUI_KEY_SPEED  7

byte uiCurrentPage=UI_PAGE_MAIN;
word speedInput;

byte readKeyboard()
// Lecture des boutons : Les boutons forment des ponts diviseurs
// à résistances et sont connectés sur une entrée analogique
// Le shield doit être modifié pour gérér un bouton supplémentaire
{
  word keyInput = analogRead(BUTTONS_ANALOG_INPUT);
  static word oldSpeedInput;
  if (keyInput > 980)
  {
    speedInput = analogRead(SPEED_ANALOG_INPUT);
    if(speedInput!=oldSpeedInput)
    {
      oldSpeedInput=speedInput;
      return DUI_KEY_SPEED;
    }
    return DUI_KEY_NONE;
  }
  if (keyInput < 50)   return DUI_KEY_RIGHT;
  if (keyInput < 180)  return DUI_KEY_UP;
  if (keyInput < 330)  return DUI_KEY_DOWN;
  if (keyInput < 530)  return DUI_KEY_LEFT;
  if (keyInput < 760)  return DUI_KEY_ESC;
  return DUI_KEY_OK;
}

// ----------------------------------------
// --- Constantes et variables          ---
// ----------------------------------------

#define UI_PAGE_MAIN_CURSOR_ADRMODE 0
#define UI_PAGE_MAIN_CURSOR_ADR0 1
#define UI_PAGE_MAIN_CURSOR_ADR1 2
#define UI_PAGE_MAIN_CURSOR_ADR2 3
#define UI_PAGE_MAIN_CURSOR_ADR3 4
#define UI_PAGE_MAIN_CURSOR_STEP 5

byte uiPageMainValAdrMode=1;
char uiPageMainValAddressString[5]="0000";
int uiPageMainValAddress=0;
byte uiPageMainValStep=3;

// ----------------------------------------
// --- Page : Pilot                     ---
// ----------------------------------------

byte dccSpeed;
char dccDir=1;
byte dccFctIndex=1;
long dccFctField=0;

void setFunction(byte index)
{
  byte type;
  word data;
  if(index<=4)
    {type=DCC_PACKET_TYPE_F0_F4;data=(dccFctField>>(DCC_FUNCTION_MAX-4))&0x1F;}
  else if(index<=8)
    {type=DCC_PACKET_TYPE_F5_F8;data=(dccFctField>>(DCC_FUNCTION_MAX-8))&0xF;}
  else if(index<=12)
    {type=DCC_PACKET_TYPE_F9_F12;data=(dccFctField>>(DCC_FUNCTION_MAX-12))&0xF;}
  if(uiPageMainValAdrMode) type|=DCC_PACKET_TYPE_ADDR_LONG;
  dccPacketFormat(type, uiPageMainValAddress, data);
}

byte setSpeedAndDir()
{
  byte type;
  switch(uiPageMainValStep)
  {
    case 0 :
      type=DCC_PACKET_TYPE_SPEED|DCC_PACKET_TYPE_STEP_14;
      dccSpeed=map(speedInput,0,1023,0,14);
      if(dccSpeed) dccSpeed++; // Pas de cran 1
      break;
    case 1 : // Idem 28 crans faute de documentation
      type=DCC_PACKET_TYPE_SPEED|DCC_PACKET_TYPE_STEP_27;
      dccSpeed=map(speedInput,0,1023,0,28);
      if(dccSpeed) dccSpeed+=3; // Pas de crans 1, 2 et 3
      break;
    case 2 :
      type=DCC_PACKET_TYPE_SPEED|DCC_PACKET_TYPE_STEP_28;
      dccSpeed=map(speedInput,0,1023,0,28);
      if(dccSpeed) dccSpeed+=3; // Pas de crans 1, 2 et 3
      break;
    case 3 :
      type=DCC_PACKET_TYPE_SPEED|DCC_PACKET_TYPE_STEP_128;
      dccSpeed=map(speedInput,0,1023,0,126);
      if(dccSpeed) dccSpeed++; // Pas de cran 1
      break;
  }
  if(uiPageMainValAdrMode) type|=DCC_PACKET_TYPE_ADDR_LONG;
  word data=(dccDir>0)?0x100:0;
  if(dccDir) data|=dccSpeed;
  dccPacketFormat(type, uiPageMainValAddress, data);
  return(dccSpeed);
}

byte uiPagePilotSpeed(byte button, byte mode)
{
  lcd.setCursor(0,1);
  byte type;
  byte speedStep=setSpeedAndDir();
  lcd.print(speedStep);
  if(dccSpeed<100) lcd.write(' ');
  if(dccSpeed<10) lcd.write(' ');
  return 0;
}

byte uiPagePilotDir(byte button, byte mode)
{
  lcd.setCursor(4,1);
  switch(button)
  {
    case DUI_KEY_LEFT :
      if(dccDir>-1) dccDir--;
      break;
    case DUI_KEY_RIGHT :
      if(dccDir<1) dccDir++;
      break;
  }
  switch(dccDir)
  {
    case 1 : lcd.write(0x7E); dccFctField|=0x1000; break;
    case 0 : lcd.write('-'); dccFctField&=~0x1000; break;
    case -1 : lcd.write(0x7F); dccFctField|=0x1000; break;
  }
  setSpeedAndDir();
  setFunction(0); // F0 = éclairage des feux
  return 0;
}

byte uiPagePilotFunction(byte button, byte mode)
{
  lcd.setCursor(8 ,1);
  switch(button)
  {
    case DUI_KEY_UP :
      if(dccFctIndex<12) dccFctIndex++;
      break;
    case DUI_KEY_DOWN :
      if(dccFctIndex>1) dccFctIndex--;
      break;
  }
  long fctBit=1<<(DCC_FUNCTION_MAX-dccFctIndex);
  long fctData=dccFctField&fctBit;
  if(button==DUI_KEY_OK)
  {
    fctData=(~fctData)&fctBit;
    dccFctField=(dccFctField&~fctBit)|fctData;
  }
  lcd.write('F');
  lcd.print(dccFctIndex);
  if(dccFctIndex<10) lcd.write(' ');
  lcd.print(fctData?F(": ON "):F(": OFF"));
  if(mode&UI_MODE_DRAW_ALL) return 0;
  if(button!=DUI_KEY_OK) return 0;
  setFunction(dccFctIndex);
  return 0;
}

byte uiPagePilot(byte button, byte mode)
{
  lcd.noBlink();
  if(mode&UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Vitesse Fonction"));
    uiPagePilotSpeed(button, mode);
    uiPagePilotDir(button, mode);
    uiPagePilotFunction(button, mode);
  }
  switch(button)
  {
    case DUI_KEY_SPEED :
      uiPagePilotSpeed(button, mode);
      break;
    case DUI_KEY_LEFT :
    case DUI_KEY_RIGHT :
      uiPagePilotDir(button, mode);
      break;
    case DUI_KEY_UP :
    case DUI_KEY_DOWN :
    case DUI_KEY_OK :
      uiPagePilotFunction(button, mode);
      break;
    case DUI_KEY_ESC :
      return(UI_PAGE_MAIN);
  }
  return 0;
}

// ----------------------------------------
// --- Page : Main                      ---
// ----------------------------------------

byte uiPageMainCursor=UI_PAGE_MAIN_CURSOR_ADRMODE;

void uiPageMainAddress(byte button, byte mode)
{
  byte index=uiPageMainCursor-UI_PAGE_MAIN_CURSOR_ADR0;
  if(mode&UI_MODE_SET_CURSOR)
  {
    lcd.setCursor(6+index,1);
    return;
  }
  lcd.setCursor(6,1);
  switch(button)
  {
    case DUI_KEY_UP :
      if(index<sizeof(uiPageMainValAddressString)-1)
      {
        uiPageMainValAddressString[index]++;
        if(uiPageMainValAddressString[index]>'9') uiPageMainValAddressString[index]='0';
      }
      break;
    case DUI_KEY_DOWN :
      if(index<sizeof(uiPageMainValAddressString)-1)
      {
        uiPageMainValAddressString[index]--;
        if(uiPageMainValAddressString[index]<'0') uiPageMainValAddressString[index]='9';
      }
      break;
    case DUI_KEY_RIGHT :
      uiPageMainCursor++;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
      if(!uiPageMainValAdrMode&&(uiPageMainCursor<UI_PAGE_MAIN_CURSOR_ADR2))
        uiPageMainCursor=UI_PAGE_MAIN_CURSOR_ADRMODE; // Adresse courte : seulement 2 digits
  }
  if(uiPageMainValAdrMode)
  {
    lcd.print(uiPageMainValAddressString);
  }
  else
  {
    lcd.print(F("  "));
    lcd.print(uiPageMainValAddressString+2);
  }
}

void uiPageMainAddrMode(byte button, byte mode)
{
  lcd.setCursor(0,1);
  if(mode&UI_MODE_SET_CURSOR)
    return;
  switch(button)
  {
    case DUI_KEY_UP :
    case DUI_KEY_DOWN :
      uiPageMainValAdrMode=!uiPageMainValAdrMode;
      break;
    case DUI_KEY_RIGHT :
      if(uiPageMainValAdrMode)
        uiPageMainCursor=UI_PAGE_MAIN_CURSOR_ADR0;
      else
        uiPageMainCursor=UI_PAGE_MAIN_CURSOR_ADR2;
  }
  lcd.print(uiPageMainValAdrMode?F("long: "):F("court:"));
  uiPageMainAddress(0,0);
}

void uiPageMainStep(byte button, byte mode)
{
  lcd.setCursor(13,1);
  if(mode&UI_MODE_SET_CURSOR)
    return;
  switch(button)
  {
    case DUI_KEY_UP :
      uiPageMainValStep++;
      if(uiPageMainValStep>3) uiPageMainValStep=0;
      break;
    case DUI_KEY_DOWN :
      uiPageMainValStep--;
      if(uiPageMainValStep>3) uiPageMainValStep=3;
      break;
    case DUI_KEY_LEFT :
      uiPageMainCursor--;
  }
  switch(uiPageMainValStep)
  {
    case 0 : lcd.print(F("14 ")); break;
    case 1 : lcd.print(F("27 ")); break;
    case 2 : lcd.print(F("28 ")); break;
    case 3 : lcd.print(F("128")); break;
  }
}

byte uiPageMain(byte button, byte mode)
{
  lcd.noBlink();
  if(mode&UI_MODE_DRAW_ALL)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print(F("Adresse    Crans"));
    uiPageMainAddrMode(button, mode);
    uiPageMainAddress(button, mode);
    uiPageMainStep(button, mode);
  }
  switch(uiPageMainCursor)
  {
    case UI_PAGE_MAIN_CURSOR_ADRMODE :
      uiPageMainAddrMode(button, mode);
      break;
    case UI_PAGE_MAIN_CURSOR_ADR0 :
    case UI_PAGE_MAIN_CURSOR_ADR1 :
    case UI_PAGE_MAIN_CURSOR_ADR2 :
    case UI_PAGE_MAIN_CURSOR_ADR3 :
      uiPageMainAddress(button, mode);
      break;
    case UI_PAGE_MAIN_CURSOR_STEP :
      uiPageMainStep(button, mode);
      break;
  }
  if(button==DUI_KEY_OK)
  {
    uiPageMainValAddress=atoi(uiPageMainValAddressString);
    dccClear();
    dccFctField=0;
    return(UI_PAGE_PILOT);
  }
  lcd.blink();
  return 0;
}

// ----------------------------------------
// --- Aiguilleur vers la bonne page    ---
// ----------------------------------------

byte uiPage(byte button, byte mode)
{
  switch(uiCurrentPage)
  {
    case UI_PAGE_MAIN : return uiPageMain(button, mode);
    case UI_PAGE_PILOT : return uiPagePilot(button, mode);
  }
}

// ----------------------------------------
// --- Code principal                   ---
// ----------------------------------------

void setup()
{
  Serial.begin(9600);
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, 0);
  delay(100);
  digitalWrite(LCD_BACKLIGHT, 1);

  dccClear();
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(DCC_OUT1, OUTPUT);
  FlexiTimer2::set(1, 0.000056, dccInterrupt);
  FlexiTimer2::start();

  lcd.begin(DISPLAY_WIDTH, DISPLAY_HEIGHT);
  lcd.noCursor();
  lcd.noBlink();
  uiPage(0, UI_MODE_DRAW_ALL);
  uiPage(0, UI_MODE_SET_CURSOR);
}

void dumpDccPackets() // Fonction de débuggage
{
  static byte counter=0;
  if(++counter==10)
  {
    counter=0;
    for(int i=0; i<DCC_PACKET_NUM; i++)
    {
      
      Serial.print(dccPacketSize[i]);
      Serial.print(" : ");
      for(int j=0; j<DCC_PACKET_SIZE; j++)
      {
        Serial.print(dccPacketData[i][j]);
        Serial.write(' ');
      }
      Serial.println();
    }
    Serial.println();
  }
}

void loop()
{
  byte key=readKeyboard();
  if(key)
  {
    byte ret=uiPage(key, 0);
    if(ret)
    {
      uiCurrentPage=ret; // Changement de page
      uiPage(0, UI_MODE_DRAW_ALL);
    }
    uiPage(0, UI_MODE_SET_CURSOR);
    while(readKeyboard());
  }
  delay(100);
//  dumpDccPackets();
}

