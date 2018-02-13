// LCD DISPLAY ALALOG
// PBA 2018-02-13
// Affiche la valeur des 6 entrées analogiques sur l'écran LCD
// Nécessite :
// - un shield LCD + clavier
// - un ou plusieurs potentiomètres branchés sur les entrées analogiques

#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

#define LCD_BACKLIGHT 10

void displayAnalogInputs() // Affiche le contenu des entrées analogiques
{
  lcd.setCursor(0, 0);
  lcd.print(analogRead(0));
  lcd.print("    ");
  lcd.setCursor(6, 0);
  lcd.print(analogRead(1));
  lcd.print("    ");
  lcd.setCursor(12, 0);
  lcd.print(analogRead(2));
  lcd.print("    ");
  lcd.setCursor(0, 1);
  lcd.print(analogRead(3));
  lcd.print("    ");
  lcd.setCursor(6, 1);
  lcd.print(analogRead(4));
  lcd.print("    ");
  lcd.setCursor(12, 1);
  lcd.print(analogRead(5));
  lcd.print("    ");
}

void setup()
{
  lcd.begin(16, 2);
  lcd.noCursor();
  lcd.noBlink();
  pinMode(LCD_BACKLIGHT, OUTPUT);
  digitalWrite(LCD_BACKLIGHT, 1);
}

void loop()
{
  displayAnalogInputs();
  delay(100);
}

