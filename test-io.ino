
#define CTRL_MUXA0 41 // Adresse 0 du multiplexeur
#define CTRL_MUXA1 43 // Adresse 1 du multiplexeur
#define CTRL_MUXA2 45 // Adresse 2 du multiplexeur
#define CTRL_MUXEN 47 // Activation du multiplexeur


void setup() {
  pinMode(CTRL_MUXEN,OUTPUT);
  pinMode(CTRL_MUXA0,OUTPUT);
  pinMode(CTRL_MUXA1,OUTPUT);
  pinMode(CTRL_MUXA2,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);

  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(CTRL_MUXEN,LOW);
  digitalWrite(CTRL_MUXA0,LOW);
  digitalWrite(CTRL_MUXA1,LOW);
  digitalWrite(CTRL_MUXA2,HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(900);

/*
  digitalWrite(CTRL_MUXEN,LOW);
  digitalWrite(CTRL_MUXA0,HIGH);
  digitalWrite(CTRL_MUXA1,LOW);
  digitalWrite(CTRL_MUXA2,HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);

  digitalWrite(CTRL_MUXEN,HIGH);
  digitalWrite(CTRL_MUXA0,LOW);
  digitalWrite(CTRL_MUXA1,HIGH);
  digitalWrite(CTRL_MUXA2,LOW);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(2000);

  digitalWrite(CTRL_MUXEN,LOW);
  digitalWrite(CTRL_MUXA0,HIGH);
  digitalWrite(CTRL_MUXA1,LOW);
  digitalWrite(CTRL_MUXA2,HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  delay(2000);
*/
}
