// TEST INTERRUPT
// PBA 2018-03-24

#include <FlexiTimer2.h>

void intFunction()
{
  static int counter=0;
  digitalWrite(LED_BUILTIN,!(counter++&0x1000));
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  FlexiTimer2::set(1, 0.000056, intFunction);
  FlexiTimer2::start();
}

void loop()
{
  delay(10000); // Rien à faire ici
}

