// TEST-CLAVIER
// PBA 2018-05-03

const byte KbdCol[4]={4,5,6,7};
const byte KbdRow[4]={8,9,10,11};
const byte KbdOut[4][4]={{'1','4','7','*'},{'2','5','8','0'},{'3','6','9','#'},{'A','B','C','D'}};

void keyboardInit(void)
{
  for(byte col=0; col<4; col++)
    pinMode(KbdCol[col],OUTPUT);
  for(byte row=0; row<4; row++)
    pinMode(KbdRow[row],INPUT_PULLUP);
}

char keyboardScan(void)
{
  for(byte col=0; col<4; col++)
  {
    for(byte col2=0; col2<4; col2++)
      digitalWrite(KbdCol[col2],HIGH);
    digitalWrite(KbdCol[col],LOW);
    for(byte row=0; row<4; row++)
      if(!digitalRead(KbdRow[row]))
        return(KbdOut[col][row]);
  }
  return(0);
}

char keyboardRead()
{
  static char lastKey=0;
  char newKey=keyboardScan();
  if(newKey!=lastKey)
  {
    delay(50);  // Anti rebond
    lastKey=newKey;
    return(newKey);
  }
  return(0);
}

void setup()
{
  Serial.begin(9600);
  keyboardInit();

}

void loop()
{
  char c = keyboardRead();
  if(c) Serial.println(c);
  delay(100);
}

