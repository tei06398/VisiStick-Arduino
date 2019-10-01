//RXD -> TX/1
//TXD -> RX/0
//GND -> GND
//VCC -> 3.3V

char bluetoothVal;           //value sent over via bluetooth
char lastVal;                //stores last state of device (on/off)
char mode='n'; //d=Data, c=Command
int count = 0;
int readings[3];
void setup()
{
 mode='n';
 Serial.begin(9600); 
 pinMode(13,OUTPUT);
}
 
 
void loop()
{
  if(Serial.available()) {
    bluetoothVal=Serial.read();
  }
  if (mode=='n') {
    Serial.println(F("Select Mode [(C)ommand or (D)ebug]:"));
    if (bluetoothVal=='c') {
      Serial.println(F("Command Mode Selected"));
      mode='c';
      lastVal=bluetoothVal;
    } else if (bluetoothVal=='d') {
      Serial.println(F("Debug Mode Selected"));
      mode='d';
      lastVal=bluetoothVal;
    } else {
      if (bluetoothVal!=lastVal) {
        Serial.println(F("Invalid Option. [(C)ommand or (D)ebug]:"));
      }
    }
    delay(2500);
  } else if (mode=='c') {
    if (bluetoothVal=='b') {
      mode='n';
      Serial.println(F("Exited Command Mode"));
      lastVal=bluetoothVal;
    }
    if (bluetoothVal=='0' && lastVal!=bluetoothVal) {
      digitalWrite(13, LOW);
      Serial.println(F("LED toggled Off"));
      lastVal=bluetoothVal;
    } else if (bluetoothVal=='1' && lastVal!=bluetoothVal) {
      digitalWrite(13, HIGH);
      Serial.println(F("LED toggled On"));
      lastVal=bluetoothVal;
    }
  } else if (mode=='d') {
    if (bluetoothVal=='b') {
      mode='n';
      Serial.println(F("Exited Debug Mode"));
      lastVal=bluetoothVal;
    }
    count++;
    Serial.println(count);
    
    delay(50);
  }
  
}
