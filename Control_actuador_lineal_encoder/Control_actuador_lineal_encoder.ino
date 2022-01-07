
// configuracion para el encoder
#define A 3
#define B 2
#define relay1 6
#define relay2 7

int ant = 0;
volatile int pos = 0;
unsigned long t;

void setup() {

  Serial.begin(9600);
  pinMode(A,INPUT);
  pinMode(B,INPUT);
  pinMode(relay1,OUTPUT);
  pinMode(relay2,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(A), encoder, LOW);
  digitalWrite(relay1, HIGH);// turn relay 1 OFF
  digitalWrite(relay2, HIGH);// turn relay 2 OFF
}

void loop() {

    if (pos != ant){
      
       Serial.println(pos);
       ant = pos;
    }
   
    if (pos >= 22){
    t = millis();
    
    while ((millis() - t) <= 5000)  {
      
    digitalWrite(relay1, LOW);// turn relay 1 ON
    digitalWrite(relay2, HIGH);// turn relay 2 OFF
   
    }
    
    delay(10000);
    t = millis();
    
    while ((millis() - t) <= 5000)  {
      
    digitalWrite(relay1, HIGH);// turn relay 1 OFF
    digitalWrite(relay2, LOW);// turn relay 2 ON
   
    }

    digitalWrite(relay1, HIGH);// turn relay 1 OFF
    digitalWrite(relay2, HIGH);// turn relay 2 OFF
    pos = 0;
    
    }
    
}


void encoder(){ // funcion para controlar el encoder

  static unsigned long ultimaInterrupcion = 0;
  unsigned long tiempoInterrupcion = millis();
  
  if (tiempoInterrupcion - ultimaInterrupcion >= 5){
    
  if(digitalRead(B) == HIGH){
    pos++; 
  }
  else {
    pos--;
  }
  
  ultimaInterrupcion = tiempoInterrupcion;
  
  }
  
  }
