// all credit goes to Random Nerd Tutorials
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33 

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 320

int x, y, z;

#define DRAW_BUF_SIZE (SCREEN_WIDTH * SCREEN_HEIGHT / 10 * (LV_COLOR_DEPTH / 8))
uint32_t draw_buf[DRAW_BUF_SIZE / 4];

static lv_obj_t * text_label_touch;
void touchscreen_read(lv_indev_t * indev, lv_indev_data_t * data) {
  if (touchscreen.tirqTouched() && touchscreen.touched()) {
    TS_Point p = touchscreen.getPoint();
    float alpha_x, beta_x, alpha_y, beta_y, delta_x, delta_y;

    // REPLACE WITH YOUR OWN CALIBRATION VALUES » https://RandomNerdTutorials.com/touchscreen-calibration/                         👍
    alpha_x = -0.001;
    beta_x = 0.086;
    delta_x = -10.224;
    alpha_y = 0.082;
    beta_y = 0.011;
    delta_y = -78.177;

    x = alpha_y * p.x + beta_y * p.y + delta_y;
    x = max(0, x);
    x = min(SCREEN_WIDTH - 1, x);

    y = alpha_x * p.x + beta_x * p.y + delta_x;
    y = max(0, y);
    y = min(SCREEN_HEIGHT - 1, y);

    // Basic Touchscreen calibration points with map function to the correct width and height
    //x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
    //y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);

    data->state = LV_INDEV_STATE_PRESSED;

    data->point.x = x;
    data->point.y = y;

    Serial.print("X = ");
    Serial.print(x);
    Serial.print(" | Y = ");
    Serial.println(y);
    String touch_data = "X = " + String(x) + "\nY = " + String(y);
    lv_label_set_text(text_label_touch, touch_data.c_str());
  }
  else data->state = LV_INDEV_STATE_RELEASED;
}


void lv_create_main_gui(void) {
  text_label_touch = lv_label_create(lv_screen_active());
  lv_label_set_text(text_label_touch, "Touch screen to test");
  lv_obj_align(text_label_touch, LV_ALIGN_CENTER, 0, 0);

  static lv_style_t style_text_label;
  lv_style_init(&style_text_label);
  lv_obj_add_style(text_label_touch, &style_text_label, 0); 
}

void setup() {
  lv_init();

  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(2);

  lv_display_t * disp;
  disp = lv_tft_espi_create(SCREEN_WIDTH, SCREEN_HEIGHT, draw_buf, sizeof(draw_buf));
  lv_display_set_rotation(disp, LV_DISPLAY_ROTATION_270);

  lv_indev_t * indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
  lv_indev_set_read_cb(indev, touchscreen_read);

  lv_create_main_gui();
}

void loop() {
  lv_task_handler();
  lv_tick_inc(5);
  delay(5);
}
