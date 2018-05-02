// LCD TEST
// PBA 2018-05-02

#include <LiquidCrystal.h>

LiquidCrystal lcd(7, 6, 5, 4, 3, 2);

#define LCD_ROWS 4
#define LCD_COLS 20

void setup()
{
  lcd.begin(LCD_COLS, LCD_ROWS);
  lcd.clear() ;
}

void loop()
{
static byte x,y; static byte c='A';
  lcd.setCursor(x,y);
  lcd.write(c++);
  if(++y==LCD_ROWS)
    {y=0; if(++x==LCD_COLS) x=0;}
  delay(20);
}

