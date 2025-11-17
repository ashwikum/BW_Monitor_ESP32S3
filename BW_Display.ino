/*
 * BW_Display.ino
 * ESP32-S3 based bandwidth gauge for Round LCD.
 * Pulls Netdata metrics over Wi-Fi and renders a dynamic LVGL dial.
 * Gauge scale adapts to instantaneous download rate while upload value
 * is reported numerically. Includes Wi-Fi reconnect and PSRAM usage.
 */
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <ctype.h>
#include <float.h>
#include <string.h>
#include <esp_heap_caps.h>
#include <lvgl.h>
#include <TFT_eSPI.h>

// --- Wi-Fi credentials (update to match your environment) ---
constexpr char WIFI_SSID[] = "Portal";
constexpr char WIFI_PASSWORD[] = "4423621021";

// --- Netdata endpoint (use the provided installation details) ---
constexpr char NETDATA_HOST[] = "192.168.128.1";
constexpr uint16_t NETDATA_PORT = 19999;
constexpr char NETDATA_CHART[] = "net.igc3";

// --- Timing configuration ---
constexpr uint32_t DATA_REFRESH_INTERVAL_MS = 1000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
constexpr uint32_t WIFI_RETRY_BACKOFF_MS = 5000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 15000;
constexpr size_t NETDATA_JSON_CAPACITY = 4096;

// --- Gauge behaviour ----------------------------------------------------
// The gauge handles instantaneous download rates up to 1 Gbps but we
// switch between four discrete scales so that low speeds remain readable.
constexpr float EASING_FACTOR = 0.18f; // unused but left for future smoothing
constexpr float GAUGE_MAX_MBPS = 1000.0f; // absolute upper bound
constexpr int GAUGE_TICK_COUNT = 60;
const float kScaleSteps[] = {10.0f, 25.0f, 100.0f, 1000.0f};
constexpr size_t kScaleStepCount = sizeof(kScaleSteps)/sizeof(kScaleSteps[0]);
float g_currentScaleMax = kScaleSteps[kScaleStepCount - 1];
size_t g_currentScaleIndex = kScaleStepCount - 1;
constexpr float NEEDLE_SMOOTHING = 1.0f; // 1.0 == disabled; needle snaps instantly

// --- LVGL display configuration ---
constexpr uint16_t SCREEN_WIDTH = 240;
constexpr uint16_t SCREEN_HEIGHT = 240;
constexpr size_t LVGL_BUFFER_PIXELS = (SCREEN_WIDTH * SCREEN_HEIGHT) / 10;

// --- Display state for LVGL ---
static lv_disp_draw_buf_t s_lvglDrawBuffer;
static lv_disp_drv_t s_lvglDisplayDriver;
static lv_color_t *s_lvglPrimaryBuffer = nullptr;
static TFT_eSPI s_tft = TFT_eSPI(SCREEN_WIDTH, SCREEN_HEIGHT);
static bool s_lvglInitialized = false;
static lv_obj_t *s_downloadLabel = nullptr;
static lv_obj_t *s_scaleLabel = nullptr;
static lv_obj_t *s_downloadUnitLabel = nullptr;
static lv_obj_t *s_uploadLabel = nullptr;
static lv_obj_t *s_speedMeter = nullptr;
static lv_meter_scale_t *s_speedScale = nullptr;
static lv_meter_indicator_t *s_speedNeedle = nullptr;
static float s_displayedDownloadMbps = 0.0f;
static lv_obj_t *s_glowArc = nullptr;

// Track instantaneous Netdata values for later animation / logging.
struct RateState
{
  float target = 0.0f;
  float display = 0.0f;
};

RateState upRate;
RateState downRate;
unsigned long lastDrawLogMillis = 0;
bool wifiDriverInitialized = false;
unsigned long g_lastWifiReconnectAttempt = 0;

unsigned long lastDataFetchMillis = 0;
unsigned long lastUpdateMillis = 0;
int consecutiveApiFailures = 0;

char wifiStatusLine[48] = "WiFi: not connected";
char fetchStatusLine[64] = "Awaiting Netdata...";
char chartStatusLine[48] = {0};

const char *kDownloadLabelCandidates[] = {
    "received", "in", "rx", "download", "down",
    "received_kilobits", "received_kbps", "recv", "recv_kbps"};
const char *kUploadLabelCandidates[] = {
    "sent", "out", "tx", "upload", "up", "transmit",
    "sent_kilobits", "sent_kbps", "send", "send_kbps"};
constexpr size_t DOWNLOAD_LABEL_COUNT = sizeof(kDownloadLabelCandidates) / sizeof(kDownloadLabelCandidates[0]);
constexpr size_t UPLOAD_LABEL_COUNT = sizeof(kUploadLabelCandidates) / sizeof(kUploadLabelCandidates[0]);

struct FormattedRate
{
  char value[12];
  char unit;
};

// --- Utility helpers --------------------------------------------------------
float clampFloat(float value, float minValue, float maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

int clampInt(int value, int minValue, int maxValue)
{
  if (value < minValue)
    return minValue;
  if (value > maxValue)
    return maxValue;
  return value;
}

bool equalsIgnoreCase(const char *a, const char *b)
{
  if (!a || !b)
    return false;
  while (*a && *b)
  {
    if (tolower(*a) != tolower(*b))
    {
      return false;
    }
    ++a;
    ++b;
  }
  return *a == *b;
}

float inferUnitMultiplier(const char *unitsFromApi)
{
  if (!unitsFromApi || unitsFromApi[0] == '\0')
  {
    return 1000.0f;
  }

  char lowered[40];
  size_t len = strlen(unitsFromApi);
  len = clampInt(static_cast<int>(len), 0, static_cast<int>(sizeof(lowered) - 1));

  for (size_t i = 0; i < len; ++i)
  {
    lowered[i] = static_cast<char>(tolower(unitsFromApi[i]));
  }
  lowered[len] = '\0';

  if (strstr(lowered, "bit"))
  {
    if (strstr(lowered, "giga"))
      return 1000.0f * 1000.0f * 1000.0f;
    if (strstr(lowered, "mega"))
      return 1000.0f * 1000.0f;
    if (strstr(lowered, "kilo"))
      return 1000.0f;
    return 1.0f;
  }

  if (strstr(lowered, "byte"))
  {
    if (strstr(lowered, "giga"))
      return 8.0f * 1000.0f * 1000.0f * 1000.0f;
    if (strstr(lowered, "mega"))
      return 8.0f * 1000.0f * 1000.0f;
    if (strstr(lowered, "kilo"))
      return 8.0f * 1000.0f;
    return 8.0f;
  }

  return 1000.0f;
}

FormattedRate formatRateForDisplay(float bps)
{
  FormattedRate result{};
  result.unit = 'b';

  const float kbps = 1000.0f;
  const float mbps = 1000.0f * kbps;
  const float gbps = 1000.0f * mbps;

  float scaled = bps;
  char unit = 'b';
  int precision = 0;

  if (bps >= gbps)
  {
    scaled = bps / gbps;
    unit = 'G';
    precision = 2;
  }
  else if (bps >= mbps)
  {
    scaled = bps / mbps;
    unit = 'M';
    precision = 2;
  }
  else if (bps >= kbps)
  {
    scaled = bps / kbps;
    unit = 'K';
    precision = 1;
  }
  else
  {
    unit = 'b';
    precision = 0;
  }

  dtostrf(scaled, 0, precision, result.value);
  result.unit = unit;
  return result;
}

int measureTextWidth(const char *text, int widthPerChar)
{
  if (!text)
  {
    return 0;
  }
  return static_cast<int>(strlen(text)) * widthPerChar;
}

// --- Wi-Fi helpers ----------------------------------------------------------
void ensureWifiDriverInitialized()
{
  if (wifiDriverInitialized)
  {
    return;
  }

  Serial.println("Initializing Wi-Fi driver (static buffers, STA mode)...");
  WiFi.setAutoReconnect(true);
  WiFi.setHostname("ESP32S3_BW_Display");
  WiFi.mode(WIFI_STA);
  wifiDriverInitialized = true;
  Serial.print("WiFi MAC Address: ");
  Serial.println(WiFi.macAddress());
}

bool attemptWifiConnect()
{
  ensureWifiDriverInitialized();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("Wi-Fi already connected.");
    return true;
  }

  Serial.print("Attempting to connect to SSID: ");
  Serial.println(WIFI_SSID);

  WiFi.disconnect(true, true);
  delay(100);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long attemptCount = 1;
  unsigned long attemptStart = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");

    if (WIFI_CONNECT_TIMEOUT_MS > 0 && millis() - attemptStart >= WIFI_CONNECT_TIMEOUT_MS)
    {
      Serial.println();
      Serial.printf("Wi-Fi attempt %lu timed out after %lu ms, retrying...\n",
                    attemptCount, millis() - attemptStart);
      attemptCount++;
      attemptStart = millis();
      WiFi.disconnect(true, true);
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
  }

  Serial.println();
  String ssid = WiFi.SSID();
  int rssi = WiFi.RSSI();
  snprintf(wifiStatusLine, sizeof(wifiStatusLine), "WiFi: %s (%d dBm)", ssid.c_str(), rssi);
  Serial.printf("Wi-Fi connected: SSID=%s RSSI=%d dBm IP=%s\n", ssid.c_str(), rssi, WiFi.localIP().toString().c_str());
  return true;
}

// --- Netdata fetch ----------------------------------------------------------
void logRateSnapshot(const char *tag)
{
  Serial.printf("[%s] up target: %.2fbps display: %.2fbps | down target: %.2fbps display: %.2fbps | uptime %lums\n",
                tag,
                upRate.target, upRate.display,
                downRate.target, downRate.display,
                millis());
}

void setFetchStatus(const char *message)
{
  if (!message)
  {
    fetchStatusLine[0] = '\0';
    return;
  }
  strncpy(fetchStatusLine, message, sizeof(fetchStatusLine) - 1);
  fetchStatusLine[sizeof(fetchStatusLine) - 1] = '\0';
}

int findLabelIndex(JsonArrayConst labels, const char *const *candidates, size_t candidateCount)
{
  if (labels.isNull() || candidateCount == 0)
  {
    return -1;
  }

  for (size_t i = 0; i < labels.size(); ++i)
  {
    const char *label = labels[i] | nullptr;
    if (!label)
    {
      continue;
    }

    for (size_t j = 0; j < candidateCount; ++j)
    {
      if (equalsIgnoreCase(label, candidates[j]))
      {
        return static_cast<int>(i);
      }
    }
  }

  return -1;
}

// Dynamically expand or reduce the gauge range so the dial always shows
// useful resolution for the current speed. We hysteresis around 50% to avoid
// oscillation between adjacent ranges.
void AdjustScaleForSample(float mbps)
{
  while (g_currentScaleIndex + 1 < kScaleStepCount && mbps > kScaleSteps[g_currentScaleIndex] * 0.5f)
  {
    g_currentScaleIndex++;
  }
  while (g_currentScaleIndex > 0 && mbps < kScaleSteps[g_currentScaleIndex - 1] * 0.5f)
  {
    g_currentScaleIndex--;
  }
  g_currentScaleMax = kScaleSteps[g_currentScaleIndex];
}
float readRowValue(JsonArrayConst row, size_t index)
{
  if (row.isNull() || index >= row.size())
  {
    return 0.0f;
  }
  return row[index] | 0.0f;
}

bool fetchNetdataSample()
{
  WiFiClient client;
  HTTPClient http;

  char url[196];
  snprintf(url, sizeof(url),
           "http://%s:%u/api/v1/data?chart=%s&after=-1&points=1&format=json",
           NETDATA_HOST, NETDATA_PORT, NETDATA_CHART);

  Serial.printf("[Netdata] Request %s @ %lu ms\n", url, millis());

  if (!http.begin(client, url))
  {
    setFetchStatus("Netdata: begin failed");
    Serial.println("[Netdata] http.begin() failed");
    return false;
  }

  http.setTimeout(4000);
  int httpCode = http.GET();

  if (httpCode != HTTP_CODE_OK)
  {
    http.end();
    ++consecutiveApiFailures;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Netdata HTTP %d (x%d)", httpCode, consecutiveApiFailures);
    setFetchStatus(buffer);
    Serial.printf("[Netdata] HTTP GET failed: code=%d message=%s\n", httpCode, HTTPClient::errorToString(httpCode).c_str());
    return false;
  }

  Serial.printf("[Netdata] OK size=%d\n", http.getSize());

  StaticJsonDocument<NETDATA_JSON_CAPACITY> doc;
  DeserializationError error = deserializeJson(doc, http.getStream());
  http.end();

  if (error)
  {
    ++consecutiveApiFailures;
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "JSON err (x%d)", consecutiveApiFailures);
    setFetchStatus(buffer);
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return false;
  }

  JsonArrayConst dataRows = doc["data"].as<JsonArrayConst>();
  JsonArrayConst labels = doc["labels"].as<JsonArrayConst>();

  if (dataRows.isNull() || dataRows.size() == 0)
  {
    ++consecutiveApiFailures;
    setFetchStatus("Netdata: no data");
    Serial.println("[Netdata] data array missing or empty");
    return false;
  }

  JsonArrayConst lastRow = dataRows[dataRows.size() - 1].as<JsonArrayConst>();
  if (lastRow.isNull())
  {
    ++consecutiveApiFailures;
    setFetchStatus("Netdata: invalid row");
    Serial.println("[Netdata] last row invalid");
    return false;
  }

  int downloadIndex = findLabelIndex(labels, kDownloadLabelCandidates, DOWNLOAD_LABEL_COUNT);
  int uploadIndex = findLabelIndex(labels, kUploadLabelCandidates, UPLOAD_LABEL_COUNT);

  bool downloadFromLabel = downloadIndex >= 0;
  bool uploadFromLabel = uploadIndex >= 0;

  if (downloadIndex < 0 && lastRow.size() > 1)
  {
    downloadIndex = 1;
  }
  if (uploadIndex < 0 && lastRow.size() > 2)
  {
    uploadIndex = 2;
  }

  if (downloadIndex < 0 || downloadIndex >= static_cast<int>(lastRow.size()) ||
      uploadIndex < 0 || uploadIndex >= static_cast<int>(lastRow.size()))
  {
    ++consecutiveApiFailures;
    setFetchStatus("Netdata: labels missing");
    Serial.printf("[Netdata] unable to select columns downloadIndex=%d uploadIndex=%d rowSize=%u\n",
                  downloadIndex, uploadIndex, static_cast<unsigned>(lastRow.size()));
    return false;
  }

  float rawDownload = readRowValue(lastRow, downloadIndex);
  float rawUpload = readRowValue(lastRow, uploadIndex);

  if (!isfinite(rawDownload) && !isfinite(rawUpload))
  {
    ++consecutiveApiFailures;
    setFetchStatus("Netdata: invalid vals");
    Serial.println("[Netdata] both rates not finite");
    return false;
  }

  if (!isfinite(rawDownload))
  {
    rawDownload = 0.0f;
  }
  if (!isfinite(rawUpload))
  {
    rawUpload = 0.0f;
  }

  if (rawUpload < 0.0f)
  {
    rawUpload = -rawUpload;
  }

  const char *unitsFromApi = doc["units"] | nullptr;
  float unitMultiplier = inferUnitMultiplier(unitsFromApi);

  downRate.target = clampFloat(rawDownload * unitMultiplier, 0.0f, FLT_MAX);
  upRate.target = clampFloat(rawUpload * unitMultiplier, 0.0f, FLT_MAX);
  logRateSnapshot("netdata");
  Serial.printf("[Netdata] down=%.2f bps up=%.2f bps units=%s\n", downRate.target, upRate.target, unitsFromApi ? unitsFromApi : "(null)");

  lastUpdateMillis = millis();
  consecutiveApiFailures = 0;
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Netdata OK (%s)", NETDATA_CHART);
  setFetchStatus(buffer);
  return true;
}

// --- LVGL display glue ------------------------------------------------------
void lvglFlushCallback(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  if (!disp || !area || !color_p)
  {
    lv_disp_flush_ready(disp);
    return;
  }

  uint32_t w = static_cast<uint32_t>(area->x2 - area->x1 + 1);
  uint32_t h = static_cast<uint32_t>(area->y2 - area->y1 + 1);

  s_tft.startWrite();
  s_tft.setAddrWindow(area->x1, area->y1, w, h);
  s_tft.pushColors(reinterpret_cast<uint16_t *>(&color_p->full), w * h, true);
  s_tft.endWrite();

  lv_disp_flush_ready(disp);
}

bool initLvgl()
{
  if (s_lvglInitialized)
  {
    return true;
  }

  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb([](const char *buf)
                           { Serial.print(buf); });
#endif

  size_t bufferBytes = LVGL_BUFFER_PIXELS * sizeof(lv_color_t);
  s_lvglPrimaryBuffer = reinterpret_cast<lv_color_t *>(heap_caps_malloc(bufferBytes, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
  if (!s_lvglPrimaryBuffer)
  {
    Serial.println("Failed to allocate LVGL draw buffer.");
    return false;
  }

  lv_disp_draw_buf_init(&s_lvglDrawBuffer, s_lvglPrimaryBuffer, nullptr, LVGL_BUFFER_PIXELS);

  s_tft.begin();
  s_tft.setRotation(0); // flip vertically

  lv_disp_drv_init(&s_lvglDisplayDriver);
  s_lvglDisplayDriver.hor_res = SCREEN_WIDTH;
  s_lvglDisplayDriver.ver_res = SCREEN_HEIGHT;
  s_lvglDisplayDriver.flush_cb = lvglFlushCallback;
  s_lvglDisplayDriver.draw_buf = &s_lvglDrawBuffer;
  lv_disp_drv_register(&s_lvglDisplayDriver);

  s_lvglInitialized = true;
  return true;
}

void formatMbps(char *buffer, size_t length, float mbps, bool includeUnits)
{
  if (!buffer || length == 0)
  {
    return;
  }

  if (includeUnits)
  {
    if (mbps >= 100.0f)
    {
      snprintf(buffer, length, "%.0f Mbps", mbps);
    }
    else
    {
      snprintf(buffer, length, "%.1f Mbps", mbps);
    }
  }
  else
  {
    if (mbps >= 100.0f)
    {
      snprintf(buffer, length, "%.0f", mbps);
    }
    else
    {
      snprintf(buffer, length, "%.1f", mbps);
    }
  }
}

// Blend between blue and green for the glow arc so that higher speeds
// appear warmer and more intense.
lv_color_t gaugeColorForRatio(float ratio)
{
  ratio = clampFloat(ratio, 0.0f, 1.0f);
  uint8_t mix = static_cast<uint8_t>(ratio * 255.0f);
  return lv_color_mix(lv_color_hex(0x9dff6f), lv_color_hex(0x23b9ff), mix);
}

void updateRateLabels(float downloadBps, float uploadBps)
{
  if (!s_lvglInitialized)
  {
    return;
  }

  float targetDownloadMbps = clampFloat(downloadBps / 1000000.0f, 0.0f, GAUGE_MAX_MBPS);
  float uploadMbps = clampFloat(uploadBps / 1000000.0f, 0.0f, GAUGE_MAX_MBPS);

  s_displayedDownloadMbps = targetDownloadMbps;
  AdjustScaleForSample(targetDownloadMbps);
  // Needle uses the adaptive scale while the numeric readout shows the
  // unscaled Mbps value so the user always sees the exact speed.
  float downloadNeedle = clampFloat(s_displayedDownloadMbps, 0.0f, g_currentScaleMax);

  if (s_speedMeter && s_speedNeedle)
  {
    lv_meter_set_scale_range(s_speedMeter, s_speedScale, 0, static_cast<int16_t>(g_currentScaleMax), 300, 120);
    lv_meter_set_indicator_value(s_speedMeter, s_speedNeedle, static_cast<int32_t>(downloadNeedle));
  }
  if (s_glowArc)
  {
    lv_arc_set_range(s_glowArc, 0, static_cast<int16_t>(g_currentScaleMax));
    lv_arc_set_value(s_glowArc, static_cast<int16_t>(downloadNeedle));
    lv_color_t col = gaugeColorForRatio(downloadNeedle / g_currentScaleMax);
    lv_obj_set_style_arc_color(s_glowArc, col, LV_PART_INDICATOR);
  }

  if (s_downloadLabel)
  {
    char text[16];
    formatMbps(text, sizeof(text), targetDownloadMbps, false);
    lv_label_set_text(s_downloadLabel, text);
  }
  if (s_scaleLabel)
  {
    char buf[24];
    snprintf(buf, sizeof(buf), "0-%.0f", g_currentScaleMax);
    lv_label_set_text(s_scaleLabel, buf);
  }
  if (s_downloadUnitLabel)
  {
    lv_label_set_text(s_downloadUnitLabel, "Mbps");
  }

  if (s_uploadLabel)
  {
    char uploadText[24];
    formatMbps(uploadText, sizeof(uploadText), uploadMbps, true);
    lv_label_set_text_fmt(s_uploadLabel, LV_SYMBOL_UPLOAD " %s", uploadText);
  }
}

void showNetdataError()
{
  if (!s_lvglInitialized)
  {
    return;
  }
  if (s_downloadLabel)
  {
    lv_label_set_text(s_downloadLabel, "--");
  }
  if (s_uploadLabel)
  {
    lv_label_set_text(s_uploadLabel, LV_SYMBOL_UPLOAD " --");
  }
  if (s_speedMeter && s_speedNeedle)
  {
    lv_meter_set_indicator_value(s_speedMeter, s_speedNeedle, 0);
  }
  if (s_glowArc)
  {
    lv_arc_set_value(s_glowArc, 0);
    lv_obj_set_style_arc_color(s_glowArc, lv_color_hex(0x23b9ff), LV_PART_INDICATOR);
  }
  s_displayedDownloadMbps = 0.0f;
}

void createRateScreen()
{
  if (!s_lvglInitialized)
  {
    return;
  }

  // Build a dark themed dial with a glow arc + LVGL meter for ticks.
  lv_obj_t *screen = lv_scr_act();
  lv_obj_set_style_bg_color(screen, lv_color_hex(0x060b15), 0);
  lv_obj_set_style_bg_grad_color(screen, lv_color_hex(0x0b1224), 0);
  lv_obj_set_style_bg_grad_dir(screen, LV_GRAD_DIR_VER, 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);

  s_glowArc = lv_arc_create(screen);
  lv_obj_set_size(s_glowArc, 220, 220);
  lv_obj_align(s_glowArc, LV_ALIGN_CENTER, 0, -10);
  lv_arc_set_rotation(s_glowArc, 120);
  lv_arc_set_bg_angles(s_glowArc, 0, 300);
  lv_arc_set_range(s_glowArc, 0, static_cast<int16_t>(GAUGE_MAX_MBPS));
  lv_arc_set_value(s_glowArc, 0);
  lv_arc_set_mode(s_glowArc, LV_ARC_MODE_NORMAL);
  lv_obj_remove_style(s_glowArc, nullptr, LV_PART_KNOB);
  lv_obj_clear_flag(s_glowArc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(s_glowArc, 14, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(s_glowArc, lv_color_hex(0x23b9ff), LV_PART_INDICATOR);
  lv_obj_set_style_arc_width(s_glowArc, 14, LV_PART_MAIN);
  lv_obj_set_style_arc_opa(s_glowArc, LV_OPA_TRANSP, LV_PART_MAIN);
  lv_obj_move_background(s_glowArc);

  s_speedMeter = lv_meter_create(screen);
  lv_obj_set_size(s_speedMeter, 210, 210);
  lv_obj_align(s_speedMeter, LV_ALIGN_CENTER, 0, -10);
  lv_obj_remove_style(s_speedMeter, nullptr, LV_PART_INDICATOR);
  lv_obj_set_style_bg_opa(s_speedMeter, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(s_speedMeter, 0, 0);

  s_speedScale = lv_meter_add_scale(s_speedMeter);
  lv_meter_set_scale_ticks(s_speedMeter, s_speedScale, GAUGE_TICK_COUNT + 1, 1, 12, lv_color_hex(0x074a3a));
  lv_meter_set_scale_major_ticks(s_speedMeter, s_speedScale, 15, 2, 20, lv_color_hex(0x7EFF8C), 0);
  lv_meter_set_scale_range(s_speedMeter, s_speedScale, 0, static_cast<int16_t>(g_currentScaleMax), 300, 120);
  lv_obj_set_style_text_font(s_speedMeter, &lv_font_montserrat_12, LV_PART_TICKS);
  lv_obj_set_style_text_color(s_speedMeter, lv_color_hex(0xbfd2ff), LV_PART_TICKS);
  lv_obj_set_style_text_opa(s_speedMeter, LV_OPA_TRANSP, LV_PART_TICKS);

  s_speedNeedle = lv_meter_add_needle_line(s_speedMeter, s_speedScale, 4, lv_color_hex(0xffa14a), -14);
  lv_obj_set_style_line_rounded(s_speedMeter, true, LV_PART_INDICATOR);

  lv_obj_t *centerCircle = lv_obj_create(screen);
  lv_obj_set_size(centerCircle, 130, 130);
  lv_obj_align(centerCircle, LV_ALIGN_CENTER, 0, -10);
  lv_obj_set_style_bg_color(centerCircle, lv_color_hex(0x050b17), 0);
  lv_obj_set_style_bg_opa(centerCircle, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(centerCircle, 2, 0);
  lv_obj_set_style_border_color(centerCircle, lv_color_hex(0x1e2c46), 0);
  lv_obj_set_style_radius(centerCircle, LV_RADIUS_CIRCLE, 0);

  s_downloadLabel = lv_label_create(screen);
  lv_label_set_text(s_downloadLabel, "--");
  lv_obj_set_style_text_font(s_downloadLabel, &lv_font_montserrat_26, 0);
  lv_obj_set_style_text_color(s_downloadLabel, lv_color_hex(0xE5F5FF), 0);
  lv_obj_align(s_downloadLabel, LV_ALIGN_CENTER, 0, -20);

  s_downloadUnitLabel = lv_label_create(screen);
  lv_label_set_text(s_downloadUnitLabel, "Mbps");
  lv_obj_set_style_text_font(s_downloadUnitLabel, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(s_downloadUnitLabel, lv_color_hex(0x82b1ff), 0);
  lv_obj_align_to(s_downloadUnitLabel, s_downloadLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);

  s_scaleLabel = lv_label_create(screen);
  lv_label_set_text(s_scaleLabel, "0-1000");
  lv_obj_set_style_text_font(s_scaleLabel, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_color(s_scaleLabel, lv_color_hex(0x5e6b96), 0);
  lv_obj_set_width(s_scaleLabel, 80);
  lv_obj_align_to(s_scaleLabel, s_downloadUnitLabel, LV_ALIGN_OUT_BOTTOM_MID, 0, 4);
  lv_obj_set_style_text_align(s_scaleLabel, LV_TEXT_ALIGN_CENTER, 0);

  s_uploadLabel = lv_label_create(screen);
  lv_label_set_text(s_uploadLabel, LV_SYMBOL_UPLOAD " --");
  lv_obj_set_style_text_font(s_uploadLabel, &lv_font_montserrat_14, 0);
  lv_obj_set_style_text_color(s_uploadLabel, lv_color_hex(0xFFB86C), 0);
  lv_obj_align(s_uploadLabel, LV_ALIGN_BOTTOM_MID, 0, -16);

  s_displayedDownloadMbps = 0.0f;
}

// --- Setup & loop -----------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  delay(50);

  Serial.println("ESP32-S3 Netdata LVGL dashboard booting...");
  snprintf(chartStatusLine, sizeof(chartStatusLine), "%s:%u %s", NETDATA_HOST, NETDATA_PORT, NETDATA_CHART);

  if (psramInit())
  {
    Serial.println("PSRAM detected.");
    Serial.print("Total PSRAM: ");
    Serial.println(ESP.getPsramSize());
    Serial.print("Free PSRAM: ");
    Serial.println(ESP.getFreePsram());
  }
  else
  {
    Serial.println("Warning: PSRAM not detected.");
  }

  if (!attemptWifiConnect())
  {
    Serial.println("Wi-Fi connection failed, continuing without network.");
  }

  if (initLvgl())
  {
    createRateScreen();
  }
  else
  {
    Serial.println("LVGL initialization failed.");
  }
}

void loop()
{
  unsigned long now = millis();

  if (now - lastDataFetchMillis >= DATA_REFRESH_INTERVAL_MS)
  {
    lastDataFetchMillis = now;
    if (WiFi.status() == WL_CONNECTED)
    {
      if (fetchNetdataSample())
      {
        updateRateLabels(downRate.target, upRate.target);
      }
      else
      {
        showNetdataError();
      }
    }
    else
    {
      Serial.println("Loop: Wi-Fi disconnected, skipping fetch.");
    }
  }

  // Keep Wi-Fi alive when Netdata is polling continuously. We retry
  // every WIFI_RECONNECT_INTERVAL_MS until the module reports WL_CONNECTED.
  if (WiFi.status() != WL_CONNECTED)
  {
    if (now - g_lastWifiReconnectAttempt >= WIFI_RECONNECT_INTERVAL_MS)
    {
      g_lastWifiReconnectAttempt = now;
      Serial.println("Attempting Wi-Fi reconnect...");
      attemptWifiConnect();
    }
  }
  else
  {
    g_lastWifiReconnectAttempt = now;
  }

  if (s_lvglInitialized)
  {
    lv_timer_handler();
  }
  delay(5);
}
