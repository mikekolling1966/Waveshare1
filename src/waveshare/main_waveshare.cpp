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

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WiFiManager.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ===========================
// --- Network Configuration ---
char sk_server_host[40] = "192.168.0.191";
const uint16_t SK_SERVER_PORT = 3000;
const char* SK_SERVER_PATH = "/signalk/v1/stream";

WebSocketsClient webSocket;

// --- Global variables ---
float g_wind_angle_deg = 0.0;
float g_wind_speed_knots = 0.0;
unsigned long last_data_received = 0;

// ===========================
// UI Elements & Functions
// ===========================
static lv_obj_t* wind_meter = NULL;
static lv_meter_indicator_t* wind_needle = NULL;
static lv_obj_t* angle_label = NULL;
static lv_obj_t* speed_label = NULL;

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

static void meter_event_cb(lv_event_t * e) {
    lv_obj_draw_part_dsc_t * dsc = lv_event_get_draw_part_dsc(e);
    if (dsc->part == LV_PART_TICKS && dsc->text != NULL) {
        if (dsc->value == 360) {
            lv_snprintf(dsc->text, dsc->text_length, "0");
        }
    }
}

void wind_create_ui(lv_obj_t* parent) {
  lv_obj_clean(parent);
  lv_obj_set_style_bg_color(parent, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(parent, LV_OPA_COVER, 0);
  lv_obj_t* title_label = lv_label_create(parent);
  lv_label_set_text(title_label, "MANXMAN");
  lv_obj_set_style_text_font(title_label, &lv_font_montserrat_28, 0);
  lv_obj_set_style_text_color(title_label, lv_color_white(), 0);
  lv_obj_align(title_label, LV_ALIGN_TOP_LEFT, 10, 10);

  // **** NEW SECTION 1: Add the Settings button ****
  lv_obj_t* config_button = lv_btn_create(parent);
  lv_obj_align(config_button, LV_ALIGN_TOP_RIGHT, -10, 10);
  lv_obj_t* icon = lv_label_create(config_button);
  lv_label_set_text(icon, LV_SYMBOL_SETTINGS);
  lv_obj_center(icon);
  void config_button_event_cb(lv_event_t * e); // Forward declaration
  lv_obj_add_event_cb(config_button, config_button_event_cb, LV_EVENT_CLICKED, NULL);

  wind_meter = lv_meter_create(parent);
  lv_obj_remove_style_all(wind_meter);
  lv_obj_set_size(wind_meter, 400, 400); 
  lv_obj_center(wind_meter);
  lv_obj_add_event_cb(wind_meter, meter_event_cb, LV_EVENT_DRAW_PART_BEGIN, NULL);
  lv_meter_scale_t* scale = lv_meter_add_scale(wind_meter);
  lv_meter_set_scale_range(wind_meter, scale, 0, 359, 360, 270);
  lv_meter_set_scale_ticks(wind_meter, scale, 37, 2, 10, lv_palette_main(LV_PALETTE_GREY));
  lv_meter_set_scale_major_ticks(wind_meter, scale, 9, 4, 20, lv_color_white(), 15);
  lv_obj_set_style_text_color(wind_meter, lv_color_white(), LV_PART_TICKS);
  lv_obj_t* zero_label = lv_label_create(parent);
  lv_obj_set_style_text_font(zero_label, lv_obj_get_style_text_font(wind_meter, LV_PART_TICKS), 0);
  lv_obj_set_style_text_color(zero_label, lv_color_white(), 0);
  lv_label_set_text(zero_label, "0");
  lv_obj_align_to(zero_label, wind_meter, LV_ALIGN_TOP_MID, 0, 35);
  lv_meter_indicator_t* arc_green = lv_meter_add_arc(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_GREEN), 0);
  lv_meter_set_indicator_start_value(wind_meter, arc_green, 1);
  lv_meter_set_indicator_end_value(wind_meter, arc_green, 180);
  lv_meter_indicator_t* arc_red = lv_meter_add_arc(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_RED), 0);
  lv_meter_set_indicator_start_value(wind_meter, arc_red, 181);
  lv_meter_set_indicator_end_value(wind_meter, arc_red, 359);
  wind_needle = lv_meter_add_needle_line(wind_meter, scale, 10, lv_palette_main(LV_PALETTE_YELLOW), -20);
  lv_obj_t* center_cover = lv_obj_create(parent);
  lv_obj_remove_style_all(center_cover);
  lv_obj_set_size(center_cover, 160, 160);
  lv_obj_set_style_bg_color(center_cover, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(center_cover, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(center_cover, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(center_cover, 0, 0);
  lv_obj_center(center_cover);
  lv_obj_t* label_cont = lv_obj_create(parent);
  lv_obj_remove_style_all(label_cont);
  lv_obj_set_size(label_cont, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_center(label_cont);
  lv_obj_set_flex_flow(label_cont, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(label_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_gap(label_cont, 10, 0);
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

// **** NEW SECTION 2: The button's event handler function ****
void config_button_event_cb(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        Serial.println("Settings button clicked. Erasing WiFi credentials and restarting into config mode...");
        WiFi.disconnect(true, true); 
        delay(100); 
        ESP.restart();
    }
}

void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.println("[WSc] Disconnected!");
      break;
    case WStype_CONNECTED: {
      Serial.println("[WSc] Connected!");
      StaticJsonDocument<256> doc;
      doc["context"] = "vessels.self";
      JsonArray subscribe = doc.createNestedArray("subscribe");
      JsonObject subscribe_awa = subscribe.createNestedObject();
      subscribe_awa["path"] = "environment.wind.angleApparent";
      subscribe_awa["period"] = 1000;
      JsonObject subscribe_aws = subscribe.createNestedObject();
      subscribe_aws["path"] = "environment.wind.speedApparent";
      subscribe_aws["period"] = 1000;
      String json_string;
      serializeJson(doc, json_string);
      webSocket.sendTXT(json_string);
      Serial.println("Subscription message sent.");
      break;
    }
    case WStype_TEXT: {
      last_data_received = millis();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, payload, length);
      JsonArray updates = doc["updates"];
      if (updates) {
        for (JsonObject update : updates) {
          JsonArray values = update["values"];
          for (JsonObject value_pair : values) {
            const char* path = value_pair["path"];
            if (!path) continue;
            if (strcmp(path, "environment.wind.angleApparent") == 0) {
              double angle_rad = value_pair["value"];
              double angle_deg = angle_rad * 180.0 / PI;
              if (angle_deg < 0) { g_wind_angle_deg = 360.0 + angle_deg; } 
              else { g_wind_angle_deg = angle_deg; }
            } else if (strcmp(path, "environment.wind.speedApparent") == 0) {
              double speed_mps = value_pair["value"];
              g_wind_speed_knots = speed_mps * 1.94384;
            }
          }
        }
      }
      break;
    }
    default:
      break;
  }
}

void show_config_message() {
    lv_obj_clean(lv_scr_act());
    lv_obj_set_style_bg_color(lv_scr_act(), lv_color_black(), 0);
    lv_obj_t* label = lv_label_create(lv_scr_act());
    lv_label_set_text(label, "No WiFi Connection\n\n"
                             "Connect to AP:\n"
                             "MANXMAN_ConfigAP\n\n"
                             "Go to 192.168.4.1 in browser");
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_28, 0);
    lv_obj_center(label);
    lv_task_handler();
}

#include <Ticker.h>
#include <HardwareSerial.h>
#define TICKER_MS 5

TCA9554 expander(EXPANDER_ADDRESS);
Arduino_DataBus *sw_spi_bus = new Arduino_SWSPI(GFX_NOT_DEFINED, TFT_CS, TFT_SCK, TFT_SDA, GFX_NOT_DEFINED);
Arduino_ESP32RGBPanel *rgbpanel = new Arduino_ESP32RGBPanel( TFT_DE, TFT_VS, TFT_HS, TFT_PCLK, TFT_R0, TFT_R1, TFT_R2, TFT_R3, TFT_R4, TFT_G0, TFT_G1, TFT_G2, TFT_G3, TFT_G4, TFT_G5, TFT_B0, TFT_B1, TFT_B2, TFT_B3, TFT_B4, TFT_HSYNC_POLARITY, TFT_HSYNC_FRONT_PORCH, TFT_HSYNC_PULSE_WIDTH, TFT_HSYNC_BACK_PORCH, TFT_VSYNC_POLARITY, TFT_VSYNC_FRONT_PORCH, TFT_VSYNC_PULSE_WIDTH, TFT_VSYNC_BACK_PORCH, TFT_PCLK_ACTIVE_NEG, TFT_DATA_SPEED, TFT_USE_BIG_ENDIAN );
Arduino_RGB_Display *tft = new Arduino_RGB_Display( TFT_WIDTH, TFT_HEIGHT, rgbpanel, ROTATION, TFT_AUTO_FLUSH, sw_spi_bus, GFX_NOT_DEFINED, st7701_type1_init_operations, sizeof(st7701_type1_init_operations) );
TAMC_GT911 touch_panel(I2C_SDA, I2C_SCL, TP_INT, -1, TFT_WIDTH, TFT_HEIGHT);
Ticker ticker;
static lv_disp_draw_buf_t draw_buffer;
static lv_color_t *frame_buffer;
static lv_disp_drv_t display_driver;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) { uint32_t w = (area->x2 - area->x1 + 1); uint32_t h = (area->y2 - area->y1 + 1); tft->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h); tft->flush(); lv_disp_flush_ready(disp); }
void ticker_call_function(void) { lv_tick_inc(TICKER_MS); }
void my_input_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) { touch_panel.read(); if (touch_panel.isTouched) { data->state = LV_INDEV_STATE_PRESSED; data->point.x = touch_panel.points[0].x; data->point.y = touch_panel.points[0].y; } else { data->state = LV_INDEV_STATE_RELEASED; } }

void setup() {
  Serial.begin(115200);

  tca_expander_reset_dance();
  ticker.attach_ms(TICKER_MS, ticker_call_function);
  
  tft->begin();
  
  touch_panel.reset();
  touch_panel.begin();
  touch_panel.setRotation(TAMC_GT911_ROTATION);
  touch_panel.setResolution(TFT_WIDTH, TFT_HEIGHT);
  
  lv_init();
  frame_buffer = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * TFT_WIDTH * TFT_HEIGHT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if(frame_buffer == NULL) { Serial.println("Frame buffer alloc failed"); while(1); }
  lv_disp_draw_buf_init(&draw_buffer, frame_buffer, NULL, TFT_WIDTH * TFT_HEIGHT);
  lv_disp_drv_init(&display_driver);
  display_driver.hor_res = TFT_WIDTH; display_driver.ver_res = TFT_HEIGHT; display_driver.flush_cb = my_disp_flush; display_driver.draw_buf = &draw_buffer;
  lv_disp_drv_register(&display_driver);
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER; indev_drv.read_cb = my_input_read;
  lv_indev_drv_register(&indev_drv);
  
  WiFiManager wm;
  WiFiManagerParameter custom_sk_host("server", "Signal K IP", sk_server_host, 40);
  wm.addParameter(&custom_sk_host);
  wm.setAPCallback([](WiFiManager* myWiFiManager) {
    show_config_message();
  });

  Serial.println("Starting WiFiManager...");
  if (!wm.autoConnect("MANXMAN_ConfigAP")) {
    Serial.println("Failed to connect and hit timeout. Restarting...");
    delay(3000);
    ESP.restart();
  }
  
  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  strcpy(sk_server_host, custom_sk_host.getValue());
  Serial.printf("Using Signal K server at: %s\n", sk_server_host);

  wind_create_ui(lv_scr_act());
  wind_update(0, 0);

  webSocket.begin(sk_server_host, SK_SERVER_PORT, SK_SERVER_PATH);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  Serial.println("Setup complete.");
}

void loop() {
  lv_task_handler();
  webSocket.loop();

  static uint32_t last_display_update = 0;
  if (millis() - last_display_update > 250) {
    last_display_update = millis();

    if (millis() - last_data_received > 5000 && last_data_received != 0) {
        lv_label_set_text(angle_label, "--- deg");
        lv_label_set_text(speed_label, "--.- kn");
    } else {
        wind_update(g_wind_angle_deg, g_wind_speed_knots);
    }
  }
}