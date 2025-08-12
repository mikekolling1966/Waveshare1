#include "Waveshare_ESP32_S3_Touch_LCD_4.h"
#include "tca_expander_reset_dance.h"
#include <Arduino.h>
#include <TCA9554.h>
#include <Arduino_GFX_Library.h>
#include <TAMC_GT911.h>
#include <PCF85063A-SOLDERED.h>
#include <ESP32time.h>
#include <driver/twai.h>
#include "can_bus_twai.h"
#include <lvgl.h>
#include <math.h>

// ===========================
// UI Elements
// ===========================
static lv_obj_t* wind_meter = NULL;
static lv_meter_indicator_t* wind_needle = NULL;
static lv_obj_t* angle_label = NULL;
static lv_obj_t* speed_label = NULL;

// This function updates the UI with new values
void wind_update(float wind_angle, float wind_speed) {
  if (wind_meter == NULL) return;

  lv_meter_set_indicator_value(wind_meter, wind_needle, (int32_t)wind_angle);
  
  char angle_buf[16];
  dtostrf(wind_angle, 3, 0, angle_buf);
  strcat(angle_buf, " deg");
  lv_label_set_text(angle_label, angle_buf);

  char speed_buf[16]; 
  dtostrf(wind_speed, 3, 1, speed_buf);
  strcat(speed_buf, " kn");
  lv_label_set_text(speed_label, speed_buf);
}

// Event handler to customize the meter's labels
static void meter_event_cb(lv_event_t * e) {
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part == LV_PART_TICKS && dsc->text != NULL) {
        if (dsc->value == 360) {
            lv_snprintf(dsc->text, dsc->text_length, "0");
        }
    }
}

// This function creates the main compass UI
void wind_create_ui(lv_obj_t* parent) {
  lv_obj_clean(parent);
  lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);

  wind_meter = lv_meter_create(parent);
  lv_obj_remove_style_all(wind_meter);
  lv_obj_set_size(wind_meter, 400, 400); 
  lv_obj_center(wind_meter);

  lv_obj_add_event_cb(wind_meter, meter_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);

  lv_meter_scale_t* scale = lv_meter_add_scale(wind_meter);
  lv_meter_set_scale_range(wind_meter, scale, 0, 360, 360, 270);
  
  lv_meter_set_scale_ticks(wind_meter, scale, 37, 2, 10, lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(wind_meter, scale, 9, 4, 20, lv_color_white(), 15);
  lv_obj_set_style_text_color(wind_meter, lv_color_white(), LV_PART_TICKS);

  lv_meter_indicator_t* arc_green = lv_meter_add_arc(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_meter_set_indicator_start_value(wind_meter, arc_green, 1);
  lv_meter_set_indicator_end_value(wind_meter, arc_green, 180);

  lv_meter_indicator_t* arc_red = lv_meter_add_arc(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_RED), 0);
  lv_meter_set_indicator_start_value(wind_meter, arc_red, 181);
  lv_meter_set_indicator_end_value(wind_meter, arc_red, 359);
  
  // Create the needle. A small negative r_mod just shortens it from the tip so it looks clean.
  wind_needle = lv_meter_add_needle_line(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_YELLOW), -20);

  // **** THE FIX: Create a black circle to cover the center of the gauge ****
  lv_obj_t* center_cover = lv_obj_create(parent);
  lv_obj_remove_style_all(center_cover); // Start with a clean object
  lv_obj_set_size(center_cover, 160, 160); // Set the size of the central blank area
  lv_obj_set_style_bg_color(center_cover, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(center_cover, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(center_cover, LV_RADIUS_CIRCLE, 0); // Make it a circle
  lv_obj_set_style_border_width(center_cover, 0, 0);
  lv_obj_center(center_cover); // Place it in the center of the screen


  // Create a container for our labels, which will sit on top of the cover
  lv_obj_t* label_cont = lv_obj_create(parent);
  lv_obj_remove_style_all(label_cont);
  lv_obj_set_size(label_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_center(label_cont);
  lv_obj_set_flex_flow(label_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(label_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_gap(label_cont, 10, 0);

  // "WIND" Title Label
lv_obj_t* wind_title_label = lv_label_create(label_cont);
lv_obj_set_style_text_font(wind_title_label, &lv_font_montserrat_48, 0);
lv_obj_set_style_text_color(wind_title_label, lv_color_white(), 0);
lv_label_set_text(wind_title_label, "WIND");

  angle_label = lv_label_create(label_cont);
  lv_obj_set_style_text_font(angle_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(angle_label, lv_color_white(), 0);
  lv_label_set_text(angle_label, "--- deg");

  speed_label = lv_label_create(label_cont);
  lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_48, 0);
  lv_obj_set_style_text_color(speed_label, lv_color_white(), 0);
  lv_label_set_text(speed_label, "--.- kn");
}

#include <Ticker.h>
#include <HardwareSerial.h>
#define TICKER_MS 5

TCA9554 expander(EXPANDER_ADDRESS);
HardwareSerial RS485(1);
Arduino_DataBus *sw_spi_bus = new Arduino_SWSPI(GFX_NOT_DEFINED, TFT_CS, TFT_SCK, TFT_SDA, GFX_NOT_DEFINED);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel(
  TFT_DE, TFT_VS, TFT_HS, TFT_PCLK,
  TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5,
  TFT_B0, TFT_B1, TFT_B2, TFT_B3, TFT_B4,
  TFT_HSYNC_POLARITY, TFT_HSYNC_FRONT_PORCH, TFT_HSYNC_PULSE_WIDTH, TFT_HSYNC_BACK_PORCH,
  TFT_VSYNC_POLARITY, TFT_VSYNC_FRONT_PORCH, TFT_VSYNC_PULSE_WIDTH, TFT_VSYNC_BACK_PORCH,
  TFT_PCLK_ACTIVE_NEG, TFT_DATA_SPEED, TFT_USE_BIG_ENDIAN
);
Arduino_RGB_Display *tft = new Arduino_RGB_Display(
  TFT_WIDTH, TFT_HEIGHT, rgbpanel, ROTATION, TFT_AUTO_FLUSH,
  sw_spi_bus, GFX_NOT_DEFINED,
  st7701_type1_init_operations, sizeof(st7701_type1_init_operations)
);
TAMC_GT911 touch_panel(I2C_SDA, I2C_SCL, TP_INT, -1, TFT_WIDTH, TFT_HEIGHT);
Ticker ticker;
PCF85063A external_rtc;
ESP32Time internal_rtc(0);
static lv_disp_draw_buf_t draw_buffer;
static lv_color_t *frame_buffer;
static lv_disp_drv_t display_driver;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    tft->flush();
    lv_disp_flush_ready(disp);
}

void ticker_call_function(void) {
  lv_tick_inc(TICKER_MS);
}

void my_input_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
  touch_panel.read();
  if (touch_panel.isTouched) {
    data->state = LV_INDEV_STATE_PRESSED;
    data->point.x = touch_panel.points[0].x;
    data->point.y = touch_panel.points[0].y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RS485_DIRECTION, OUTPUT);
  digitalWrite(RS485_DIRECTION, HIGH);
  RS485.begin(RS485_BITRATE, SERIAL_8N1, RS485_RECEIVE_PIN, -1);
  tca_expander_reset_dance();
  ticker.attach_ms(TICKER_MS, ticker_call_function);
  external_rtc.begin();
  if(external_rtc.getYear() < 2025) {
    external_rtc.setTime(13, 55, 00);
    external_rtc.setDate(2, 1, 4, 2025);
  }
  internal_rtc.setTime(external_rtc.getSecond(), external_rtc.getMinute(), external_rtc.getHour(), external_rtc.getDay(), external_rtc.getMonth(), external_rtc.getYear());
  
  tft->begin();
  tft->flush();
  for (uint16_t x_coord = 0; x_coord < TFT_WIDTH; x_coord++) {
    for (uint16_t y_coord = 0; y_coord < TFT_HEIGHT; y_coord++) {
      tft->writePixel(x_coord, y_coord, tft->color565(x_coord << 1, (x_coord + y_coord) << 2, y_coord << 1));
    }
  }
  tft->flush();
  delay(2500);

  touch_panel.reset();
  touch_panel.begin();
  touch_panel.setRotation(TAMC_GT911_ROTATION);
  touch_panel.setResolution(TFT_WIDTH, TFT_HEIGHT);
  
  lv_init();
  frame_buffer = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * TFT_WIDTH * TFT_HEIGHT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if(frame_buffer == NULL) {
    Serial.println("Frame buffer alloc failed");
    while(1);
  }
  lv_disp_draw_buf_init(&draw_buffer, frame_buffer, NULL, TFT_WIDTH * TFT_HEIGHT);
  lv_disp_drv_init(&display_driver);
  display_driver.hor_res = TFT_WIDTH;
  display_driver.ver_res = TFT_HEIGHT;
  display_driver.flush_cb = my_disp_flush;
  display_driver.draw_buf = &draw_buffer;
  lv_disp_drv_register(&display_driver);
  
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_input_read;
  lv_indev_drv_register(&indev_drv);
  
  wind_create_ui(lv_scr_act());

  Serial.printf("Setup complete. Available PSRAM: %d KB\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM)>>10);
}

void loop() {
  lv_task_handler();

  static float angle = 0;
  static float speed = 0;
  
  static uint32_t last_update = 0;
  if (millis() - last_update > 100) {
    last_update = millis();
    
    angle = fmodf(angle + 5.0f, 360.0f);
    speed = fmodf(speed + 0.7f, 25.0f);
    
    wind_update(angle, speed);
  }

  if (RS485.available() > 0) {
    Serial.write(RS485.read());
  }
}