// TEST-SERVO
// PBA 2018-04-17

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_MIN 200
#define SERVO_MAX 350

// Définit un circuit PCA9685 configuré en adresse 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void changeAigPos(int toPos, byte moveSpeed)
{
  static int pos = SERVO_MIN;
  if(!moveSpeed)
    pwm.setPWM(0, 0,pos=toPos); // Déplacement instantané
  else if(pos<toPos)
  {
    for(; pos<=toPos; pos++)
    {
      pwm.setPWM(0, 0,pos); // Déplacement lent
      delay (moveSpeed);
    }
  }  
  else if(pos>toPos)
  {
    for(; pos>=toPos; pos--)
    {
      pwm.setPWM(0, 0,pos); // Déplacement lent
      delay (moveSpeed);
    }
  }  
}

void setup()
{
  Serial.begin(9600);
  pwm.begin();        // Initialisation du circuit
  pwm.setPWMFreq(50); // Fréquence de fonctionnement
  delay(10);
  changeAigPos(SERVO_MIN,0); // Position de départ
}

void loop()
{
  changeAigPos(SERVO_MAX,0);
  delay(1000);
  changeAigPos(SERVO_MIN,0);
  delay(1000);
}

