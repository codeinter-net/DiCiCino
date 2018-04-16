// TEST AIGUILLAGES ELECTRO-AIMANTS
// PBA 2018-04-15

// assignation de la matrice aux sorties de l'Arduino
byte eaRows[8]={2,3,4,5,6,7,8,9};
byte eaCols[8]={10,11,12,13,A0,A1,A2,A3};

#define EA_PULSE 50 // Durée de l'impulsion envoyée à l'électro-aimant

void eaSwitch(byte row,byte col)
{
  digitalWrite(eaRows[row],HIGH); // Activation d'une ligne
  digitalWrite(eaCols[col],HIGH); // et d'une colonne
  delay(EA_PULSE);
  digitalWrite(eaRows[row],LOW);  // Désactivation
  digitalWrite(eaCols[col],LOW);
}

void setup()
{
  byte i;
  for(i=0;i<8;i++)
  {
    pinMode(eaRows[i],OUTPUT); // Toutes les lignes de la matrice en sortie
    digitalWrite(eaRows[i],LOW);
    pinMode(eaCols[i],OUTPUT);
    digitalWrite(eaCols[i],LOW);
  }
}

void loop()
{
  eaSwitch(0,0);
  delay(2000);
  eaSwitch(1,0);
  delay(2000);
  eaSwitch(0,1);
  delay(2000);
  eaSwitch(1,1);
  delay(2000);
}

