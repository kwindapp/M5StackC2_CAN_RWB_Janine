#include <Arduino.h>
#include <M5Unified.h>

#include <WiFi.h>
#include <esp_now.h>

#include <lvgl.h>
#include "driver/twai.h"
#include "ui_font_Hollow85.h"
#include "ui_font_Hollow22.h"
#include "ui_font_Hollow38.h"
#include "ui_font_t20.h"
/***************************************************
 * CONFIG / FLAGS
 ***************************************************/
bool testMode = true;   // true = simulate values, false = use CAN bus

// Peer MACs
uint8_t peerCarDash[] = { 0xD0, 0xCF, 0x13, 0x1E, 0x14, 0x6C };
uint8_t macLilygo[]   = { 0x30, 0x30, 0xF9, 0x34, 0x7A, 0x78 };

// CAN pins (adjust if needed)
#define CAN_TX_GPIO GPIO_NUM_5
#define CAN_RX_GPIO GPIO_NUM_4

// Trijekt / AIM CAN IDs
const uint32_t ID_A = 0x770;   // AIM stream ID
const uint32_t ID_B = 0x780;   // unused in AIM mode but kept for compatibility

// Old style IDs (no longer used in AIM mode, but constants kept)
const uint32_t ID_RPM       = ID_A;
const uint32_t ID_STATUS_B  = ID_A + 0x01;
const uint32_t ID_TEMP_A    = ID_A + 0x02;
const uint32_t ID_BATT_VOLT = ID_B + 0x01;
const uint32_t ID_FUNC_STATUS = ID_A + 0x03;

// bit flags
const uint8_t FUNC_BYTE_INDEX = 0;
const uint8_t FUNC_PIN38_MASK = 0x01;

// scaling (used only in testMode now)
const float RPM_FACTOR      = 0.9f;
const float MOTOR_FACTOR    = 0.1f;
const float BATT_FACTOR     = 0.001f;
const float THROTTLE_FACTOR = 0.1f;

const int MAX_RPM = 8000;
const uint32_t RPM_TIMEOUT_MS = 500;

uint32_t lastRpmUpdateMs = 0;
bool funk_active = false;

/***************************************************
 * ECU DATA STRUCT
 ***************************************************/
struct ECUData {
  uint16_t rpm;
  float batt;
  float motor;          // hier aktuell "Motor temp" (bei dir eher Öl / Ersatz)
  float throttle_dk;
  float throttle_pedal;
  float lambda;         // NEU: Lambda (gemisch λ)

  // optional für später:
  // float map_mbar;    // Manifold pressure
  // float air_temp;    // Ansaugluft-Temp
  // float fuel_temp;   // Kraftstofftemp
  // uint16_t err_flags;
} ecu;

/***************************************************
 * AIM full-packet reconstruction (35 bytes)
 ***************************************************/
static uint8_t aim_ring[35];
static uint8_t aim_packet[35];
static uint8_t aim_write_pos = 0;

// helpers: big-endian
static inline int16_t S16BE_bytes(const uint8_t *p) {
  return (int16_t)((uint16_t(p[0]) << 8) | uint16_t(p[1]));
}
static inline uint16_t U16BE_bytes(const uint8_t *p) {
  return (uint16_t(p[0]) << 8) | uint16_t(p[1]);
}

/***************************************************
 * ESP-NOW PACKET STRUCT
 ***************************************************/
typedef struct {
  uint16_t rpm;
  float batt;
  float motor;
  float dk;
  float gp;
  uint8_t funk;
} DashPacket;

/***************************************************
 * LVGL CONFIG
 ***************************************************/
#define LV_HOR_RES_MAX 320
#define LV_VER_RES_MAX 240

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[LV_HOR_RES_MAX * 40];

static lv_obj_t *label_rpm;
static lv_obj_t *label_row1;
static lv_obj_t *label_row2;
static lv_obj_t *label_row3;   // Lambda
static lv_obj_t *shadow_row1;
static lv_obj_t *shadow_row2;
static lv_obj_t *shadow_row3;
static lv_obj_t *bar_rpm;

/***************************************************
 * HELPERS
 ***************************************************/
uint16_t U16BE(const twai_message_t &m, int o) {
  return (uint16_t(m.data[o]) << 8) | m.data[o + 1];
}

/***************************************************
 * LVGL DISPLAY PORT
 ***************************************************/
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  M5.Display.startWrite();
  M5.Display.setAddrWindow(area->x1, area->y1, w, h);
  M5.Display.pushColors((uint16_t *)&color_p->full, w * h, true);
  M5.Display.endWrite();

  lv_disp_flush_ready(disp);
}

void lv_port_init() {
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LV_HOR_RES_MAX * 40);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LV_HOR_RES_MAX;
  disp_drv.ver_res = LV_VER_RES_MAX;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.flush_cb = my_disp_flush;
  lv_disp_drv_register(&disp_drv);
}

/***************************************************
 * UI CREATION
 ***************************************************/
void create_ui() {
  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  // RPM label
  label_rpm = lv_label_create(scr);
  lv_label_set_text(label_rpm, "0");
  lv_obj_align(label_rpm, LV_ALIGN_TOP_MID, 0, 5);
  lv_obj_set_style_text_color(label_rpm, lv_color_white(), 0);
  lv_obj_set_style_text_font(label_rpm, &ui_font_Hollow85, LV_PART_MAIN | LV_STATE_DEFAULT);

  // RPM bar
  bar_rpm = lv_bar_create(scr);
  lv_bar_set_range(bar_rpm, 0, 8);
  lv_bar_set_value(bar_rpm, 0, LV_ANIM_OFF);
  lv_obj_set_size(bar_rpm, 280, 20);
  lv_obj_align(bar_rpm, LV_ALIGN_CENTER, 0, -20);

  // bar background style
  lv_obj_set_style_bg_color(bar_rpm, lv_color_hex(0x303030), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(bar_rpm, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_radius(bar_rpm, 8, LV_PART_MAIN);

  // bar indicator style (green → red)
  lv_obj_set_style_radius(bar_rpm, 8, LV_PART_INDICATOR);
  lv_obj_set_style_bg_color(bar_rpm, lv_color_hex(0x00FF40), LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_color(bar_rpm, lv_color_hex(0xFF0000), LV_PART_INDICATOR);
  lv_obj_set_style_bg_grad_dir(bar_rpm, LV_GRAD_DIR_HOR, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(bar_rpm, LV_OPA_COVER, LV_PART_INDICATOR);

  //-----------------------------------------
  // Row 1 (Battery & Motor) + SHADOW
  //-----------------------------------------
  shadow_row1 = lv_label_create(scr);
  lv_label_set_text(shadow_row1, "Batt: --.-V  Motor: --.-C");
  lv_obj_align(shadow_row1, LV_ALIGN_BOTTOM_MID, 2, -88);   // slight offset
  lv_obj_set_style_text_color(shadow_row1, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(shadow_row1, &ui_font_t20, LV_PART_MAIN);

  label_row1 = lv_label_create(scr);
  lv_label_set_text(label_row1, "Batt: --.-V  Motor: --.-C");
  lv_obj_align(label_row1, LV_ALIGN_BOTTOM_MID, 0, -90);
  lv_obj_set_style_text_color(label_row1, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(label_row1, &ui_font_t20, LV_PART_MAIN);
  lv_label_set_recolor(label_row1, true);   // enable recolor for values

  //-----------------------------------------
  // Row 2 (DK / GP / Funk) + SHADOW
  //-----------------------------------------
  shadow_row2 = lv_label_create(scr);
  lv_label_set_text(shadow_row2, "DK: --.-%  GP: --.-% : OFF");
  lv_obj_align(shadow_row2, LV_ALIGN_BOTTOM_MID, 2, 2);
  lv_obj_set_style_text_color(shadow_row2, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(shadow_row2, &ui_font_t20, LV_PART_MAIN);

  label_row2 = lv_label_create(scr);
  lv_label_set_text(label_row2, "DK: --.-%  GP: --.-% :OFF");
  lv_obj_align(label_row2, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_set_style_text_color(label_row2, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(label_row2, &ui_font_t20, LV_PART_MAIN);
  lv_label_set_recolor(label_row2, true);

  //-----------------------------------------
  // Row 3 (Lambda) + SHADOW
  //-----------------------------------------
  shadow_row3 = lv_label_create(scr);
  lv_label_set_text(shadow_row3, "Lambda: --.--");
  lv_obj_align(shadow_row3, LV_ALIGN_BOTTOM_MID, 2, -48);
  lv_obj_set_style_text_color(shadow_row3, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(shadow_row3, &ui_font_t20, LV_PART_MAIN);

  label_row3 = lv_label_create(scr);
  lv_label_set_text(label_row3, "Lambda: --.--");
  lv_obj_align(label_row3, LV_ALIGN_BOTTOM_MID, 0, -50);
  lv_obj_set_style_text_color(label_row3, lv_color_hex(0xFFFFFF), 0);
  lv_obj_set_style_text_font(label_row3, &ui_font_t20, LV_PART_MAIN);
  lv_label_set_recolor(label_row3, true);
}

/***************************************************
 * UI UPDATE
 ***************************************************/
void update_ui() {
  char buf[96];
  char bufPlain[96];
  char bufColor[96];

  // RPM
  snprintf(buf, sizeof(buf), "%u", ecu.rpm);
  lv_label_set_text(label_rpm, buf);

  int barVal = (ecu.rpm + 500) / 1000;
  if (barVal < 0) barVal = 0;
  if (barVal > 8) barVal = 8;
  lv_bar_set_value(bar_rpm, barVal, LV_ANIM_OFF);

  // ---------------- Row 1: Batt / Motor ----------------
  // Plain (for shadow)
  snprintf(bufPlain, sizeof(bufPlain),
           "Batt: %.1fV  Motor: %.1fC",
           ecu.batt, ecu.motor);

  // Colored values:
  // Batt = yellow, Motor = orange
  snprintf(bufColor, sizeof(bufColor),
           "Batt: #FFFF00 %.1fV#  Motor: #FFA500 %.1fC#",
           ecu.batt, ecu.motor);

  lv_label_set_text(shadow_row1, bufPlain);
  lv_label_set_text(label_row1,  bufColor);

  // ---------------- Row 2: DK / GP / Funk ----------------
  const char *funkTextPlain = funk_active ? "ON" : "OFF";
  const char *funkTextColor = funk_active ? "#00FF00 ON#" : "#FF0000 OFF#";

  // Plain (shadow)
  snprintf(bufPlain, sizeof(bufPlain),
           "DK: %.1f%%  GP: %.1f%%  : %s",
           ecu.throttle_dk,
           ecu.throttle_pedal,
           funkTextPlain);

  // Colored: DK blue, GP yellow, Funk green/red
  snprintf(bufColor, sizeof(bufColor),
           " DK: #00BFFF %.1f%%#  GP: #FFFF00 %.1f%%#  : %s",
           ecu.throttle_dk,
           ecu.throttle_pedal,
           funkTextColor);

  lv_label_set_text(shadow_row2, bufPlain);
  lv_label_set_text(label_row2,  bufColor);

  // ---------------- Row 3: Lambda ----------------
  // Choose color by lambda value:
  // rich (<0.95) = orange, ok (0.95–1.05) = green, lean (>1.05) = blue
  const char *lambdaColor;
  if (ecu.lambda < 0.95f) {
    lambdaColor = "#FF8000";   // rich
  } else if (ecu.lambda > 1.05f) {
    lambdaColor = "#00BFFF";   // lean
  } else {
    lambdaColor = "#00FF00";   // ok
  }

  // Plain (shadow)
  snprintf(bufPlain, sizeof(bufPlain),
           "Lambda: %.2f",
           ecu.lambda);

  // Colored value
  snprintf(bufColor, sizeof(bufColor),
           "Lambda: %s %.2f#",
           lambdaColor,
           ecu.lambda);

  lv_label_set_text(shadow_row3, bufPlain);
  lv_label_set_text(label_row3,  bufColor);
}

/***************************************************
 * CAN INIT
 ***************************************************/
bool can_init() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI install failed");
    return false;
  }

  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start failed");
    return false;
  }

  Serial.println("TWAI (CAN) initialized");
  return true;
}

/***************************************************
 * AIM PACKET → ECU DECODE
 ***************************************************/
void aim_decode_packet_to_ecu(const uint8_t *p) {
  // RPM
  uint16_t rpm_raw = U16BE_bytes(&p[0]);
  if (rpm_raw > MAX_RPM) rpm_raw = MAX_RPM;
  ecu.rpm = rpm_raw;
  lastRpmUpdateMs = millis();

  // "Water" / Motor-Temp (bei dir evtl. als Ersatzsensor benutzt)
  float waterC = S16BE_bytes(&p[8]) * 0.1f;
  ecu.motor = (ecu.motor < 0.01f)
                ? waterC
                : ecu.motor + 0.10f * (waterC - ecu.motor); // smoothing

  // Battery
  int16_t batt_raw = S16BE_bytes(&p[12]);
  float battV = (batt_raw * 5.0f / 1024.0f); // voltage am ADC
  battV *= 3.0f;                             // ~1:3 Teiler (anpassen falls nötig)
  ecu.batt = (ecu.batt < 0.01f)
               ? battV
               : ecu.batt + 0.15f * (battV - ecu.batt);

  // Throttle (DK & GP)
  float thr = U16BE_bytes(&p[14]) * 0.1f;   // 0.1 % per bit
  ecu.throttle_dk     = thr;
  ecu.throttle_pedal  = thr;

  // Lambda (aktiv)
  ecu.lambda = U16BE_bytes(&p[22]) * 0.001f;  // 0.001 λ per bit
}

/***************************************************
 * AIM PACKET ASSEMBLER FOR ID 0x770
 ***************************************************/
static uint32_t can_rx_count = 0;

void decodeAIM_770(const twai_message_t &m) {
  const uint8_t *data = m.data;

  for (int i = 0; i < m.data_length_code; ++i) {
    uint8_t b = data[i];

    // write byte into 35-byte ring buffer
    aim_ring[aim_write_pos] = b;
    int idx_last  = aim_write_pos;
    int idx_prev1 = (idx_last + 35 - 1) % 35;
    int idx_prev2 = (idx_last + 35 - 2) % 35;
    aim_write_pos = (aim_write_pos + 1) % 35;

    // look for marker sequence: FC, FB, FA (bytes 31,32,33)
    if (aim_ring[idx_prev2] == 0xFC &&
        aim_ring[idx_prev1] == 0xFB &&
        aim_ring[idx_last]  == 0xFA) {

      // idx_prev2 ist physischer Index von logischem byte31
      int start_idx = (idx_prev2 + 35 - 31) % 35;

      // rebuild ordered packet 0..34
      for (int j = 0; j < 35; ++j) {
        int src = (start_idx + j) % 35;
        aim_packet[j] = aim_ring[src];
      }

      // Optional debug: erste paar Pakete
      if (can_rx_count < 5) {
        Serial.print("AIM PACKET RAW: ");
        for (int j = 0; j < 35; ++j) {
          if (aim_packet[j] < 0x10) Serial.print("0");
          Serial.print(aim_packet[j], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }

      // decode into ecu
      aim_decode_packet_to_ecu(aim_packet);
    }
  }

  can_rx_count++;
}

/***************************************************
 * CAN DECODE DISPATCH
 ***************************************************/
void decodeTrijekt(const twai_message_t &m) {
  if (testMode) return;

  if (m.identifier == ID_A) {   // 0x770, AIM protocol Type 1
    decodeAIM_770(m);
    return;
  }
}

/***************************************************
 * ESP-NOW CALLBACK
 ***************************************************/
void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send → ");
  for (int i = 0; i < 6; i++) {
    if (i) Serial.print(":");
    Serial.printf("%02X", mac_addr[i]);
  }
  Serial.print(" : ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

/***************************************************
 * ADD PEER
 ***************************************************/
void addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (!esp_now_is_peer_exist(mac)) {
    if (esp_now_add_peer(&peer) == ESP_OK) {
      Serial.print("Added peer: ");
      for (int i = 0; i < 6; i++) {
        if (i) Serial.print(":");
        Serial.printf("%02X", mac[i]);
      }
      Serial.println();
    } else {
      Serial.println("Failed to add peer!");
    }
  }
}

/***************************************************
 * ESP-NOW INIT
 ***************************************************/
void initEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  Serial.print("Sender MAC: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("esp_now_init FAILED");
    return;
  }

  esp_now_register_send_cb(onEspNowSent);

  addPeer(peerCarDash);
  addPeer(macLilygo);
}

/***************************************************
 * ESP-NOW SEND
 ***************************************************/
void sendEspNow() {
  DashPacket p;
  p.rpm   = ecu.rpm;
  p.batt  = ecu.batt;
  p.motor = ecu.motor;
  p.dk    = ecu.throttle_dk;
  p.gp    = ecu.throttle_pedal;
  p.funk  = funk_active ? 1 : 0;

  esp_now_send(peerCarDash, (uint8_t*)&p, sizeof(p));
  esp_now_send(macLilygo, (uint8_t*)&p, sizeof(p));
}

/***************************************************
 * SETUP
 ***************************************************/
void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  M5.Display.setRotation(1);
  M5.Display.fillScreen(TFT_BLACK);

  Serial.begin(115200);
  delay(200);

  lv_port_init();
  create_ui();

  ecu.rpm = 0;
  ecu.batt = 13.8f;
  ecu.motor = 80.0f;
  ecu.throttle_dk = 0;
  ecu.throttle_pedal = 0;
  ecu.lambda = 1.00f;
  funk_active = false;

  lastRpmUpdateMs = millis();

  if (!testMode) {
    if (!can_init()) {
      testMode = true;
      Serial.println("CAN init failed → switching to testMode");
    }
  }

  initEspNow();
}

/***************************************************
 * LOOP
 ***************************************************/
void loop() {
  static uint32_t lastUI = 0;
  static uint32_t lastSend = 0;
  static uint32_t lastTick = 0;

  uint32_t now = millis();

  // LVGL tick
  lv_tick_inc(now - lastTick);
  lastTick = now;

  if (testMode) {
    static int rpmSim = 0;
    static int dir = 1;
    rpmSim += dir * 10;
    if (rpmSim >= MAX_RPM) { rpmSim = MAX_RPM; dir = -1; }
    if (rpmSim <= 0)       { rpmSim = 0;       dir = 1; }

    ecu.rpm = rpmSim;
    ecu.batt = 13.8f;
    ecu.motor = random(90, 100);
    ecu.throttle_dk = random(90, 100);
    ecu.throttle_pedal = ecu.throttle_dk;
    ecu.lambda = 0.90f + (random(-5, 6) / 100.0f);  // 0.85–0.95 zum Testen
    funk_active = random(0, 2);
  } else {
    twai_message_t m;
    if (twai_receive(&m, pdMS_TO_TICKS(5)) == ESP_OK) {
      decodeTrijekt(m);
    }
    if (millis() - lastRpmUpdateMs > RPM_TIMEOUT_MS)
      ecu.rpm = 0;
  }

  if (now - lastUI > 100) {
    update_ui();
    lastUI = now;
  }

  if (now - lastSend > 100) {
    sendEspNow();
    lastSend = now;
  }

  lv_timer_handler();
  delay(5);
}
