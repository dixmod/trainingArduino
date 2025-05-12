#include "wifi_config.h"
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

// #include <Fonts/GFXFF/FreeSans9pt7b.h>

constexpr int SCREEN_SIZE = 240;
constexpr int CENTER_X = SCREEN_SIZE / 2;
constexpr int CENTER_Y = SCREEN_SIZE / 2;
constexpr int RADIUS = 100;  // Максимально большой радиус с отступом


Adafruit_BME280 bme;
unsigned long lastSensorUpdate = 0;
const int SENSOR_UPDATE_INTERVAL = 60000; // Обновление каждые 5 секунд


TFT_eSPI tft = TFT_eSPI();

int prevHourX = -1, prevHourY = -1;
int prevMinX = -1, prevMinY = -1;
int prevSecX = -1, prevSecY = -1;

void setup() {
  initializeBacklight();
  initializeDisplay();
  initializeBME280(); // Инициализация датчика

  // testFonts();

  drawClockFace();

  Serial.begin(115200);
  delay(1000); // Дать время Serial подключиться

  connectToWiFi();
  synchronizeTime();

  Serial.println("Setup complete.");

  disconnectWiFi();
}

void loop() {
  struct tm timeinfo;

  if (!getLocalTime(&timeinfo)) {
    Serial.println("[ERROR] Failed to obtain time from NTP.");
    delay(1000);
    return;
  }

  int hours = timeinfo.tm_hour % 12;
  int minutes = timeinfo.tm_min;
  int seconds = timeinfo.tm_sec;

  drawClockHands(hours, minutes, seconds);
  updateSensorData(); // Обновление показаний датчика

  delay(1000);
}

void initializeBME280() {
  if (!bme.begin(0x76)) { // Адрес 0x76 или 0x77
    Serial.println("Could not find a valid BME280 sensor!");
    while (1);
  }
  Serial.println("[INFO] BME280 sensor initialized");
}

// Обновление и отображение данных с датчика
void updateSensorData() {
  if (millis() - lastSensorUpdate > SENSOR_UPDATE_INTERVAL) {
    float temp = bme.readTemperature();
    float humidity = bme.readHumidity();
    float pressure = bme.readPressure() / 100.0F;

    // tft.setTextFont(1);
    // tft.setTextColor(TFT_WHITE, TFT_BLACK);
    
    // Вывод температуры
    // tft.setCursor(10, SCREEN_SIZE - 30);
    // tft.printf("Temp: %.1fC", temp);
    Serial.printf("Temp: %.1fC, Humi: %.1f%%, Pres: %.1fhPa\n", temp, humidity, pressure);
    
    // Вывод влажности
    // tft.setCursor(100, SCREEN_SIZE - 30);
    // tft.printf("Humi: %.1f%%", humidity);
    
    // Вывод давления
    // tft.setCursor(10, SCREEN_SIZE - 15);
    // tft.printf("Pres: %.1fhPa", pressure);

    lastSensorUpdate = millis();
  }
}

void initializeBacklight() {
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  Serial.println("[INFO] Backlight pin initialized and turned ON.");
}

void initializeDisplay() {
  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  Serial.println("[INFO] Display initialized and cleared.");
}

void connectToWiFi() {
  Serial.printf("[INFO] Connecting to WiFi SSID: %s\n", ssid);
  WiFi.begin(ssid, password);

  int retryCount = 0;
  int animSecond = 0; // Для анимации секундной стрелки

  while (WiFi.status() != WL_CONNECTED) {
    // Рисуем секундную стрелку с текущим значением animSecond
    drawClockHands(0, 0, animSecond);

    animSecond = (animSecond + 1) % 60;

    delay(200); // Частота обновления анимации (5 кадров в секунду)

    Serial.print(".");

    retryCount++;
    if (retryCount > 60) { // 30 секунд ожидания
      Serial.println("\n[ERROR] WiFi connection timeout.");
      return;
    }
  }
  Serial.println("\n[INFO] WiFi connected.");
  Serial.printf("[INFO] IP address: %s\n", WiFi.localIP().toString().c_str());
}

void disconnectWiFi() {
  WiFi.disconnect(true);  // Отключаем и очищаем настройки Wi-Fi
  WiFi.mode(WIFI_OFF);    // Выключаем Wi-Fi модуль
  Serial.println("[INFO] WiFi disconnected and turned off.");
}

void synchronizeTime() {
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("[INFO] Waiting for NTP time synchronization...");

  struct tm dummyTime;
  int retryCount = 0;
  int animSecond = 0; // Для анимации секундной стрелки как секундомера

  while (!getLocalTime(&dummyTime)) {
    // Рисуем секундную стрелку с текущим значением animSecond
    drawClockHands(0, 0, animSecond);

    animSecond = (animSecond + 1) % 60; // Увеличиваем секунду, циклично от 0 до 59

    delay(200); // Частота обновления анимации (5 кадров в секунду)

    Serial.print(".");

    retryCount++;
    if (retryCount > 150) { // Около 30 секунд ожидания
      Serial.println("\n[ERROR] NTP time sync timeout.");
      return;
    }
  }

  Serial.println("\n[INFO] Time synchronized.");
}

void drawClockFace() {
  tft.fillScreen(tft.color565(30, 30, 30)); // тёмно-серый фон

  uint16_t darkBronze = tft.color565(80, 50, 20);
  uint16_t lightBronze = tft.color565(205, 127, 50);

  int darkBronzeRadius = RADIUS - 25;  // уменьшенный радиус

  // Объёмный циферблат с уменьшенным кольцом darkBronze
  tft.fillCircle(CENTER_X, CENTER_Y, darkBronzeRadius, darkBronze);
  tft.fillCircle(CENTER_X, CENTER_Y, darkBronzeRadius - 25, lightBronze);

  tft.drawCircle(CENTER_X, CENTER_Y, darkBronzeRadius, tft.color565(150, 100, 30));
  tft.drawCircle(CENTER_X, CENTER_Y, darkBronzeRadius - 25, tft.color565(220, 180, 80));

  // Римские цифры
  uint16_t shadowColor = tft.color565(30, 20, 10);
  uint16_t bronzeColor = tft.color565(230, 180, 70);

  const char* romanNumerals[12] = {
    "I", "II", "III", "IV", "V",
    "VI", "VII", "VIII", "IX", "X", "XI", "XII"
  };

  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextDatum(MC_DATUM);

  int numeralRadius = RADIUS-5;

  for (int i = 0; i < 12; i++) {
    float angle = (i * 30 - 60) * DEG_TO_RAD;

    int x = CENTER_X + numeralRadius * cos(angle);
    int y = CENTER_Y + numeralRadius * sin(angle);

    // Тень
    tft.setTextColor(shadowColor);
    tft.drawString(romanNumerals[i], x + 2, y + 2);

    // Основной текст
    tft.setTextColor(bronzeColor);
    tft.drawString(romanNumerals[i], x, y);
  }

  for (int i = 0; i < 60; i++) {
    float angle = i * 6 * DEG_TO_RAD;
    int diffStart = 0;

    if (i % 5 != 0) {
      diffStart = 12;
    }

    int xStart = CENTER_X + (darkBronzeRadius - diffStart) * sin(angle);
    int yStart = CENTER_Y - (darkBronzeRadius - diffStart) * cos(angle);

    int xEnd = CENTER_X + (darkBronzeRadius - 25) * sin(angle);
    int yEnd = CENTER_Y - (darkBronzeRadius - 25) * cos(angle);

    // Тень рисок (если нужно)
    //tft.drawLine(xStart + 1, yStart + 1, xEnd + 1, yEnd + 1, tft.color565(50, 30, 10));

    // Основная линия рисок
    tft.drawLine(xStart, yStart, xEnd, yEnd, tft.color565(230, 180, 70));

    // Очистка области для датчиков
  //  tft.fillRect(0, SCREEN_SIZE - 40, SCREEN_SIZE, 40, TFT_BLACK);
  }

  Serial.println("[INFO] Full screen volume clock face drawn.");
}



void drawClockHands(int hours, int minutes, int seconds) {
  uint16_t bgColor = tft.color565(205, 127, 50); // Цвет циферблата (фон для стирания)

  // Стираем старые стрелки (тень и основную линию)
  if (prevHourX >= 0 && prevHourY >= 0) {
    // Стираем тень часовой стрелки
    tft.drawLine(CENTER_X + 2, CENTER_Y + 2, prevHourX + 2, prevHourY + 2, bgColor);
    // Стираем основную часовую стрелку (3 линии для толщины)
    tft.drawLine(CENTER_X, CENTER_Y, prevHourX, prevHourY, bgColor);
    tft.drawLine(CENTER_X - 1, CENTER_Y, prevHourX - 1, prevHourY, bgColor);
    tft.drawLine(CENTER_X + 1, CENTER_Y, prevHourX + 1, prevHourY, bgColor);
  }

  if (prevMinX >= 0 && prevMinY >= 0) {
    // Стираем тень минутной стрелки
    tft.drawLine(CENTER_X + 2, CENTER_Y + 2, prevMinX + 2, prevMinY + 2, bgColor);
    // Стираем основную минутную стрелку (3 линии)
    tft.drawLine(CENTER_X, CENTER_Y, prevMinX, prevMinY, bgColor);
    tft.drawLine(CENTER_X - 1, CENTER_Y, prevMinX - 1, prevMinY, bgColor);
    tft.drawLine(CENTER_X + 1, CENTER_Y, prevMinX + 1, prevMinY, bgColor);
  }

  if (prevSecX >= 0 && prevSecY >= 0) {
    // Стираем тень секундной стрелки
    //tft.drawLine(CENTER_X + 2, CENTER_Y + 2, prevSecX + 2, prevSecY + 2, bgColor);
    // Стираем основную секундную стрелку (2 линии для толщины)
    tft.drawLine(CENTER_X, CENTER_Y, prevSecX, prevSecY, bgColor);
    tft.drawLine(CENTER_X + 1, CENTER_Y, prevSecX + 1, prevSecY, bgColor);
  }

  // Часовая стрелка (длиннее и толще)
  float hourAngle = ((hours % 12) + minutes / 60.0f) * 30 * DEG_TO_RAD;
  int hourX = CENTER_X + (RADIUS - 60) * sin(hourAngle);
  int hourY = CENTER_Y - (RADIUS - 60) * cos(hourAngle);

  // Рисуем тень часовой стрелки (сдвиг +2,+2)
  tft.drawLine(CENTER_X + 2, CENTER_Y + 2, hourX + 2, hourY + 2, tft.color565(80, 50, 20));
  // Основная часовая стрелка (3 линии для толщины)
  tft.drawLine(CENTER_X, CENTER_Y, hourX, hourY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X - 1, CENTER_Y, hourX - 1, hourY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X + 1, CENTER_Y, hourX + 1, hourY, tft.color565(230, 180, 70));

  // Минутная стрелка (длиннее и толще)
  float minAngle = (minutes + seconds / 60.0f) * 6 * DEG_TO_RAD;
  int minX = CENTER_X + (RADIUS - 50) * sin(minAngle);
  int minY = CENTER_Y - (RADIUS - 50) * cos(minAngle);

  // Тень минутной стрелки
  tft.drawLine(CENTER_X + 2, CENTER_Y + 2, minX + 2, minY + 2, tft.color565(80, 50, 20));
  // Основная минутная стрелка (3 линии)
  tft.drawLine(CENTER_X, CENTER_Y, minX, minY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X - 1, CENTER_Y, minX - 1, minY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X + 1, CENTER_Y, minX + 1, minY, tft.color565(230, 180, 70));

  // Секундная стрелка (толще и длиннее)
  float secAngle = seconds * 6 * DEG_TO_RAD;
  int secX = CENTER_X + (RADIUS - 35) * sin(secAngle);
  int secY = CENTER_Y - (RADIUS - 35) * cos(secAngle);

  // Тень секундной стрелки
  //tft.drawLine(CENTER_X + 2, CENTER_Y + 2, secX + 2, secY + 2, tft.color565(80, 50, 20));
  // Основная секундная стрелка (2 линии)
  tft.drawLine(CENTER_X, CENTER_Y, secX, secY, TFT_RED);
  tft.drawLine(CENTER_X + 1, CENTER_Y, secX + 1, secY, TFT_RED);

  // Центр циферблата
  tft.fillCircle(CENTER_X, CENTER_Y, 5, TFT_WHITE);

  // Сохраняем координаты для стирания
  prevHourX = hourX;
  prevHourY = hourY;
  prevMinX = minX;
  prevMinY = minY;
  prevSecX = secX;
  prevSecY = secY;
}

