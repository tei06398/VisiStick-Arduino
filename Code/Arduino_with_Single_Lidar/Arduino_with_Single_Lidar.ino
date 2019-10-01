void setup()
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  //RED -> 5V
  //BLACK -> GND
  //YELLOW -> A0
}

void loop()
{
  digitalWrite(13, HIGH);
  int val = analogRead(A0); // read the sensor (inches)
  Serial.println(exp(8.5841 - log(val)));
  delay(50);
}
