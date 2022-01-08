// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int lectura[3]; //lectura de los sensores

String msg; // mensaje recibido
int msg1; // mensaje recibido int
void setup() {
  
  //pines para sensores
  pinMode(10, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(14, INPUT); // Sets the echoPin as an INPUT
  pinMode(11, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(15, INPUT); // Sets the echoPin as an INPUT
  pinMode(12, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(16, INPUT); // Sets the echoPin as an INPUT

  //pines para motores traseros
  pinMode(9, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);

  //pines para motores delanteros
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(2, OUTPUT);


  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop() {
  // put your main code here, to run repeatedly:

msg1 = readSerialPort();

if (msg1 == 1){

  digitalWrite(9, HIGH);
  
}

for(int i = 10; i<13; i++){
lectura[i-10] = leersonido(i,i+4);
}

sendData(lectura[0],lectura[1],lectura[2]);
}

void sendData(int msg0,int msg1,int msg2) {
  //write data
  Serial.println((String)msg0+","+(String)msg1+","+(String)msg2);
} 


int readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
      msg1 = msg.toFloat();
      return msg1;
    }
    Serial.flush();
  }
}

int leersonido(int trigPin,int echoPin){

    // Clears the trigPin condition
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)

  return distance;
}
