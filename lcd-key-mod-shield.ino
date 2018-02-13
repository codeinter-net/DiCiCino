// LCD KEY MOD SHIELD
// PBA 2018-02-13
// Mise en oeuvre d'un shield écran + clavier modifié (le bouton RESET devient OK)
// Nécessite :
// - un shield LCD + clavier
// - un potentiomètre branché sur l'entrée A1

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define KEY_NONE   0
#define KEY_UP     1
#define KEY_DOWN   2
#define KEY_LEFT   3
#define KEY_RIGHT  4
#define KEY_SELECT 5
#define KEY_ESC    5 // Nouveau nom pour la touche SELECT
#define KEY_RST    6
#define KEY_OK     6 // Nouveau nom pour la touche RESET
#define LCD_BACKLIGHT 10

// 3 caractères programmables pour dessiner un curseur semi-graphique
const byte ProgChar[8][8]=
{
  {B10000,B10000,B10000,B10000,B10000,B10000,B10000,B10000}, // Trait vertical à gauche
  {B00100,B00100,B00100,B00100,B00100,B00100,B00100,B00100}, // Trait vertical au milieu
  {B00001,B00001,B00001,B00001,B00001,B00001,B00001,B00001}, // Trait vertical à droite
};

byte readButtons()  // Lecture des boutons
{
  word keyInput = analogRead(0);  // En commentaire : les valeurs lues lors des tests
  if (keyInput > 950) return KEY_NONE;   // 1019
  if (keyInput < 50)   return KEY_RIGHT; // 0
  if (keyInput < 180)  return KEY_UP;    // 98
  if (keyInput < 330)  return KEY_DOWN;  // 256
  if (keyInput < 530)  return KEY_LEFT;  // 411
  if (keyInput < 760)  return KEY_ESC;   // 644
  return KEY_OK;
}

void displayButtons() // Affiche le nom du bouton sur lequel on appuie
{
  lcd.setCursor(0, 0);
  switch(readButtons())
  {
  case KEY_UP :
    lcd.print("UP   ");
    break;
  case KEY_DOWN :
    lcd.print("DOWN ");
    break;
  case KEY_LEFT :
    lcd.print("LEFT ");
    break;
  case KEY_RIGHT :
    lcd.print("RIGHT");
    break;
  case KEY_ESC :
    lcd.print("ESC  ");
    break;
  case KEY_OK :
    lcd.print("OK   ");
    break;
  default :
    lcd.print("     ");
  }
}

void displayAnalogData() // Affiche la position du potentiomètre sous la forme d'un curseur
{
  word data = analogRead(1);
  static word oldData = 1024; // Précédente valeur lue
  if(data!=oldData)  // Ne met à jour l'affichage qu'en cas de changement
  {
    lcd.setCursor(oldData>>6, 1); // Effacement de l'ancien curseur
    lcd.write(' ');
    lcd.setCursor(data>>6, 1); // Les 4 bits de poids fort positionnent le caractère
    word sub = data&0x3F;
    lcd.write(sub/22); // Les 6 bits de poids faible, divisés par 22 donnent un chiffre entre 0 et 2
    oldData=data;      // ... ce  chiffre correspond au caractère programmable à afficher
  }
}

void setup()
{
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.noBlink();
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, 1);

  // Définition des caractères programmables (codes 0, 1 et 2)
  int i;
  for(i=0; i<3; i++)
    lcd.createChar(i,ProgChar[i]);
}

void loop()
{
  displayButtons();
  displayAnalogData();
  delay(100);
}

