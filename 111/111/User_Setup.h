#define GC9A01_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 240
#define TFT_CS   5   // Пин CS вашего дисплея
#define TFT_DC   2   // Пин DC
#define TFT_RST  4   // Пин RES (если не подключен - оставьте -1)
#define TFT_BL   15  // Пин BLK (подсветка, если управляется)
#define TFT_MOSI 23  // Пин SDA
#define TFT_SCLK 18  // Пин SCL

#define TFT_BGR  // Для 90% дисплеев GC9A01
#define SPI_FREQUENCY  10000000 // Снизьте до 10 МГц для теста
// #define TFT_RGB_ORDER TFT_RGB  // или TFT_BGR