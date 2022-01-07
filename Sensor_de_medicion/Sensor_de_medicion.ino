#include <SoftwareSerial.h>

byte n, p, k; 
const byte nitro[] = {0x01, 0x03, 0x00,0x1e,0x00, 0x01, 0xe4, 0x0c};
const byte fos[] = {0x01, 0x03, 0x00,0x1f,0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01, 0x03, 0x00,0x20,0x00, 0x01, 0x85, 0xc0};

byte values[11];
SoftwareSerial mod(10,11);

void setup() {
  
  mod.begin(9600);
  Serial.begin(9600);

}

void loop() {

  n = nitrogeno();
  delay(250);
  p = fosforo();
  delay(250);
  k = potasio();
  delay(250);
  
  Serial.print("N: ");
  Serial.println(n);
  
  Serial.print("P: ");
  Serial.println(p);
  
  Serial.print("K: ");
  Serial.println(k);
  Serial.println();
  delay(3000);
}

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
