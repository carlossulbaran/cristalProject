String msg = ""; // mensaje recibido
int msgv;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed
  pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  readSerialPort();
Serial.println(msgv);
if (msgv == 1){
digitalWrite(5, HIGH);

}
else if (msgv == 0){
digitalWrite(5, LOW);
}

}


//Leer data de la RPI
void readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
      msgv = msg.toInt();
      
    }
    Serial.flush();
  }
}
