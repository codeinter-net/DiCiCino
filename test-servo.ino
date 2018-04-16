// TEST-SERVO
// PBA 2018-04-16

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Définit un circuit PCA9685 configuré en adresse 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setup()
{
  Serial.begin(9600);
  pwm.begin();        // Initialisation du circuit
  pwm.setPWMFreq(50); // Fréquence de fonctionnement
  delay(10);
}

void loop()
{
  int pos=(analogRead(A1)>>1); // Valeur entre 0 et 511
  pwm.setPWM(0, 0,pos); // Fixe la valeur du servo
  Serial.println(pos);
  delay (100);
}

