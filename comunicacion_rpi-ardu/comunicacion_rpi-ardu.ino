// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int lectura[3]; //lectura de los sensores


void setup() {

  pinMode(10, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(14, INPUT); // Sets the echoPin as an INPUT
  pinMode(11, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(15, INPUT); // Sets the echoPin as an INPUT
  pinMode(12, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(16, INPUT); // Sets the echoPin as an INPUT
  
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop() {
  // put your main code here, to run repeatedly:

for(int i = 10; i<13; i++){
lectura[i-10] = leersonido(i,i+4);
}

sendData(lectura[0],lectura[1],lectura[2]);
}

void sendData(int msg0,int msg1,int msg2) {
  //write data
  Serial.println((String)msg0+","+(String)msg1+","+(String)msg2);
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
