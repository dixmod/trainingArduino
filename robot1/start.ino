
#define EEP 2

#define IN_LEFT_FRONT 9
#define IN_LEFT_BACK 8

#define IN_RIGHT_FRON 11
#define IN_RIGHT_BACK 10

void setup() {
  // put your setup code here, to run once:
  pinMode(2,1);
  
  pinMode(IN_LEFT_FRONT,1);
  pinMode(IN_LEFT_BACK,1);
  
  pinMode(IN_RIGHT_FRON,1);
  pinMode(IN_RIGHT_BACK,1);
}

void loop() {
  delay(5000);
  
  digitalWrite(EEP,1);
  
  digitalWrite(IN_LEFT_FRONT,1);
  digitalWrite(IN_LEFT_BACK,0);
  
  digitalWrite(IN_RIGHT_FRON,1);
  digitalWrite(IN_RIGHT_BACK,0);
  
  delay(5000);

  digitalWrite(IN_LEFT_FRONT,0);
  digitalWrite(IN_LEFT_BACK,0);
  
  digitalWrite(IN_RIGHT_FRON,0);
  digitalWrite(IN_RIGHT_BACK,1);

  delay(5000);

  digitalWrite(IN_LEFT_FRONT,0);
  digitalWrite(IN_LEFT_BACK,1);
  
  digitalWrite(IN_RIGHT_FRON,0);
  digitalWrite(IN_RIGHT_BACK,0);

  delay(5000);

  digitalWrite(IN_LEFT_FRONT,0);
  digitalWrite(IN_LEFT_BACK,1);
  
  digitalWrite(IN_RIGHT_FRON,0);
  digitalWrite(IN_RIGHT_BACK,1);

  delay(5000);
}
