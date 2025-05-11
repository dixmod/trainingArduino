#include "wifi_config.h"
#include <TFT_eSPI.h>
#include <WiFi.h>
#include <time.h>

constexpr int SCREEN_SIZE = 240;
constexpr int CENTER_X = SCREEN_SIZE / 2;
constexpr int CENTER_Y = SCREEN_SIZE / 2;
constexpr int RADIUS = 110;  // Максимально большой радиус с отступом

TFT_eSPI tft = TFT_eSPI();

int prevHourX = -1, prevHourY = -1;
int prevMinX = -1, prevMinY = -1;
int prevSecX = -1, prevSecY = -1;

void setup() {
  Serial.begin(115200);
  delay(1000); // Дать время Serial подключиться

  initializeBacklight();
  initializeDisplay();

  drawClockFace();

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

  delay(1000);
}

void initializeBacklight() {
  pinMode(15, OUTPUT);
  digitalWrite(15, HIGH);
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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
  // Установка часового пояса (GMT+3) и серверов NTP
  configTime(3 * 3600, 0, "pool.ntp.org", "time.nist.gov");
  Serial.println("[INFO] Waiting for NTP time synchronization...");

  struct tm timeinfo;
  int retryCount = 0;
  while (!getLocalTime(&timeinfo)) {
    delay(500);
    Serial.print(".");
    retryCount++;
    
    if (retryCount > 60) { // 30 секунд ожидания
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

  // Объёмный циферблат
  tft.fillCircle(CENTER_X, CENTER_Y, RADIUS, darkBronze);
  tft.fillCircle(CENTER_X, CENTER_Y, RADIUS - 10, lightBronze);

  tft.drawCircle(CENTER_X, CENTER_Y, RADIUS, tft.color565(150, 100, 30));
  tft.drawCircle(CENTER_X, CENTER_Y, RADIUS - 10, tft.color565(220, 180, 80));

  // Римские цифры
  uint16_t shadowColor = tft.color565(30, 20, 10);
  uint16_t bronzeColor = tft.color565(230, 180, 70);

  const char* romanNumerals[12] = {
    "XII", "I", "II", "III", "IV", "V",
    "VI", "VII", "VIII", "IX", "X", "XI"
  };

  tft.setTextFont(2);
  tft.setTextSize(3);  // Увеличим размер шрифта для больших цифр

  for (int i = 0; i < 12; i++) {
    float angle = (i * 30 - 60) * DEG_TO_RAD;

    int x = CENTER_X + (RADIUS - 35) * cos(angle);  // Смещаем цифры чуть дальше от центра
    int y = CENTER_Y + (RADIUS - 35) * sin(angle);

    // Тень
    tft.setTextColor(shadowColor);
    tft.setCursor(x + 2, y + 2);
    tft.print(romanNumerals[i]);

    // Основной текст
    tft.setTextColor(bronzeColor);
    tft.setCursor(x, y);
    tft.print(romanNumerals[i]);
  }

  // Метки часов с тенью
  for (int i = 0; i < 12; i++) {
    float angle = i * 30 * DEG_TO_RAD;

    int xStart = CENTER_X + (RADIUS - 15) * sin(angle);
    int yStart = CENTER_Y - (RADIUS - 15) * cos(angle);
    int xEnd = CENTER_X + RADIUS * sin(angle);
    int yEnd = CENTER_Y - RADIUS * cos(angle);

    // Тень
    tft.drawLine(xStart + 1, yStart + 1, xEnd + 1, yEnd + 1, tft.color565(50, 30, 10));
    // Основная линия
    tft.drawLine(xStart, yStart, xEnd, yEnd, tft.color565(230, 180, 70));
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
    tft.drawLine(CENTER_X + 2, CENTER_Y + 2, prevSecX + 2, prevSecY + 2, bgColor);
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
  int minX = CENTER_X + (RADIUS - 40) * sin(minAngle);
  int minY = CENTER_Y - (RADIUS - 40) * cos(minAngle);

  // Тень минутной стрелки
  tft.drawLine(CENTER_X + 2, CENTER_Y + 2, minX + 2, minY + 2, tft.color565(80, 50, 20));
  // Основная минутная стрелка (3 линии)
  tft.drawLine(CENTER_X, CENTER_Y, minX, minY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X - 1, CENTER_Y, minX - 1, minY, tft.color565(230, 180, 70));
  tft.drawLine(CENTER_X + 1, CENTER_Y, minX + 1, minY, tft.color565(230, 180, 70));

  // Секундная стрелка (толще и длиннее)
  float secAngle = seconds * 6 * DEG_TO_RAD;
  int secX = CENTER_X + (RADIUS - 30) * sin(secAngle);
  int secY = CENTER_Y - (RADIUS - 30) * cos(secAngle);

  // Тень секундной стрелки
  tft.drawLine(CENTER_X + 2, CENTER_Y + 2, secX + 2, secY + 2, tft.color565(80, 50, 20));
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


