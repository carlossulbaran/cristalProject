#include <SoftwareSerial.h>

byte n, p, k;
const byte nitro[] = {0x01, 0x03, 0x00,0x1e,0x00, 0x01, 0xe4, 0x0c};
const byte fos[] = {0x01, 0x03, 0x00,0x1f,0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01, 0x03, 0x00,0x20,0x00, 0x01, 0x85, 0xc0};

byte values[11];
SoftwareSerial mod(50,51);

int npk[3] = {0,0,0};

// defines variables
long duration; // variable for the duration of sound wave travel
int distance; // variable for the distance measurement

int lectura[3]; //lectura de los sensores

String msg; // mensaje recibido
int msg1; // mensaje recibido int


int cont;

//controlar arduino
int x;
int y;
int v;

//valores velocidad motores
int z[2] = {0,0};
int msgv;

//velocidad motores
int vel_der;
int vel_iz;

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

  //pin para ponerse modo muestreo, ultrasonidos o motores
  pinMode(13, INPUT);
  pinMode(17, INPUT);

  //controlar el recibimiento de data
  pinMode(21, INPUT);

  mod.begin(9600);
  Serial.begin(9600); // // Serial Communication is starting with 9600 of baudrate speed

}

void loop() {
  // put your main code here, to run repeatedly:

x = digitalRead(13);
y = digitalRead(17);

Serial.println(x);
Serial.println(y);


//modo ultrasonico
if (x == 1 && y == 0){
  
for(int i = 10; i<13; i++){
lectura[i-10] = leersonido(i,i+4);
}

sendData(lectura[0],lectura[1],lectura[2]);
}

//modo muestreo
else if (x == 0 && y == 1){
  n = nitrogeno();
  delay(250);
  p = fosforo();
  delay(250);
  k = potasio();
  
  npk[0] = n;
  npk[1] = p;
  npk[2] = k;

  Serial.flush();
  
  Serial.println((String)npk[0]+","+(String)npk[1]+","+(String)npk[2]);
  delay(3000);

}

//modo conducir motores
else if (x == 1 && y == 1){

cont = 0;

while (cont < 2){
  
v = digitalRead(21);


Serial.flush();

if (v == 0 && cont == 0){
z[0] = readSerialPort();
cont = cont+1;

}

else if (v == 1 && cont == 1){
z[1] = readSerialPort();
cont = cont+1;

}
else{

}
// z[0]=der,z[1]=iz
}

Serial.println("normal");
 analogWrite(2,z[0]);
analogWrite(3,LOW);
analogWrite(4,z[1]);
analogWrite(5,LOW);

analogWrite(6,z[0]);
analogWrite(7,LOW);
analogWrite(8,z[1]);
analogWrite(9,LOW);


}

}

//enviar data a la RPI
void sendData(int msg0,int msg1,int msg2) {
  //write data
  Serial.flush();
  Serial.println((String)msg0+","+(String)msg1+","+(String)msg2);
} 

//Leer data de la RPI
int readSerialPort() {
  msg = "";
  if (Serial.available()) {
    delay(10);
    while (Serial.available() > 0) {
      msg += (char)Serial.read();
      msgv = msg.toInt();
    }
    Serial.flush();
    return msgv;
  }
}

//Leer ultrasonido
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

//leer sensor de npk
byte nitrogeno(){
 
  if (mod.write(nitro,sizeof(nitro))==8){
    
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
      
      }
     
    }
  return values[4];
  
}

byte fosforo(){
 
  if (mod.write(fos,sizeof(fos))==8){
    
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
      
      }
      
    }
  return values[4];
  
}

byte potasio(){
 
  if (mod.write(pota,sizeof(pota))==8){
    
    for(byte i=0;i<7;i++){
      values[i] = mod.read();
      
      }
      
    }
  return values[4];
  
}
