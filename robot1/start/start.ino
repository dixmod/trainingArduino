
// drv8833
#define EEP 2
#define IN_LEFT_FRONT 9
#define IN_LEFT_BACK 8
#define IN_RIGHT_FRON 11
#define IN_RIGHT_BACK 10

// hc-sr04
#define PIN_TRIG 13
#define PIN_ECHO 12

long duration, cm;

void setup() {
  Serial.begin (9600);
  
  pinMode(EEP, 1);

  pinMode(IN_LEFT_FRONT, 1);
  pinMode(IN_LEFT_BACK, 1);

  pinMode(IN_RIGHT_FRON, 1);
  pinMode(IN_RIGHT_BACK, 1);


  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);

  //delay(5000);
}

void loop() {
  digitalWrite(EEP, 1);

  float duration = getDuration();
  
  Serial.print("Расстояние до объекта: ");
  Serial.print(duration);
  Serial.println(" см."); 
  
  if (duration > 3) {
    goFront();
  } else {
    goBack();
    goBackRigh();
  }
}

long getDuration() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);

  // Выставив высокий уровень сигнала, ждем около 10 микросекунд. В этот момент датчик будет посылать сигналы с частотой 40 КГц.
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  //  Время задержки акустического сигнала на эхолокаторе.
  duration = pulseIn(PIN_ECHO, HIGH);

  // Теперь осталось преобразовать время в расстояние
  cm = (duration / 2) / 29.1;

  return cm;
}


void goFront() {
  digitalWrite(IN_LEFT_FRONT, 1);
  digitalWrite(IN_LEFT_BACK, 0);

  digitalWrite(IN_RIGHT_FRON, 1);
  digitalWrite(IN_RIGHT_BACK, 0);

  delay(1000);
}

void goBack() {
  digitalWrite(IN_LEFT_FRONT, 0);
  digitalWrite(IN_LEFT_BACK, 1);

  digitalWrite(IN_RIGHT_FRON, 0);
  digitalWrite(IN_RIGHT_BACK, 1);

  delay(1000);
}

void goBackLeft() {
  digitalWrite(IN_LEFT_FRONT, 0);
  digitalWrite(IN_LEFT_BACK, 1);

  digitalWrite(IN_RIGHT_FRON, 0);
  digitalWrite(IN_RIGHT_BACK, 0);

  delay(1000);
}

void goBackRigh() {
  digitalWrite(IN_LEFT_FRONT, 0);
  digitalWrite(IN_LEFT_BACK, 0);

  digitalWrite(IN_RIGHT_FRON, 0);
  digitalWrite(IN_RIGHT_BACK, 1);

  delay(1000);
}
