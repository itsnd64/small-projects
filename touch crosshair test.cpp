#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#define XPT2046_IRQ  36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK  25
#define XPT2046_CS   33

TFT_eSPI tft = TFT_eSPI(TFT_WIDTH, TFT_HEIGHT);
TFT_eSprite sprite = TFT_eSprite(&tft);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

void setup() {
  Serial.begin(115200);

  tft.init();
  tft.setRotation(1);
  sprite.setColorDepth(8);
  sprite.createSprite(tft.width(), tft.height());

  SPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI);
  ts.begin();
  ts.setRotation(1);
}

void loop() {
  sprite.fillSprite(TFT_BLACK);
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    int x, y;
    float alpha_x, beta_x, alpha_y, beta_y, delta_x, delta_y;

    alpha_x = -0.001;
    beta_x = 0.086;
    delta_x = -10.224;
    alpha_y = 0.082;
    beta_y = 0.011;
    delta_y = -78.177;

    x = alpha_y * p.x + beta_y * p.y + delta_y;
    x = max(0, x);
    x = min(tft.width() - 1, x);

    y = alpha_x * p.x + beta_x * p.y + delta_x;
    y = max(0, y);
    y = min(tft.height() - 1, y);

    sprite.fillRect(0, y, tft.width(), 1, TFT_WHITE);
    sprite.fillRect(x, 0, 1, tft.height(), TFT_WHITE);
  }
  sprite.pushSprite(0, 0);
  delay(10);
}
