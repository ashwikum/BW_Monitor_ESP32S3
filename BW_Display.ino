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
constexpr char NETDATA_CHART[] = "system.net";

// --- Timing configuration ---
constexpr uint32_t DATA_REFRESH_INTERVAL_MS = 1000;
constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 30000;
constexpr uint32_t WIFI_RETRY_BACKOFF_MS = 5000;
constexpr size_t NETDATA_JSON_CAPACITY = 4096;

// --- Gauge behaviour ---
constexpr float EASING_FACTOR = 0.18f; // smoothing factor for animation

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
static lv_obj_t *s_uploadLabel = nullptr;

struct RateState
{
  float target = 0.0f;
  float display = 0.0f;
};

RateState upRate;
RateState downRate;
unsigned long lastDrawLogMillis = 0;
bool wifiDriverInitialized = false;

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
  s_tft.setRotation(2); // flip vertically

  lv_disp_drv_init(&s_lvglDisplayDriver);
  s_lvglDisplayDriver.hor_res = SCREEN_WIDTH;
  s_lvglDisplayDriver.ver_res = SCREEN_HEIGHT;
  s_lvglDisplayDriver.flush_cb = lvglFlushCallback;
  s_lvglDisplayDriver.draw_buf = &s_lvglDrawBuffer;
  lv_disp_drv_register(&s_lvglDisplayDriver);

  s_lvglInitialized = true;
  return true;
}

void setRateLabel(lv_obj_t *label, const char *prefix, float bps)
{
  if (!label)
  {
    return;
  }

  FormattedRate rate = formatRateForDisplay(bps);
  char unitSuffix[8];
  if (rate.unit == 'b')
  {
    strcpy(unitSuffix, "bps");
  }
  else
  {
    snprintf(unitSuffix, sizeof(unitSuffix), "%cbps", rate.unit);
  }

  char text[48];
  snprintf(text, sizeof(text), "%s: %s %s", prefix, rate.value, unitSuffix);
  lv_label_set_text(label, text);
}

void updateRateLabels(float downloadBps, float uploadBps)
{
  if (!s_lvglInitialized)
  {
    return;
  }
  setRateLabel(s_downloadLabel, "Down", downloadBps);
  setRateLabel(s_uploadLabel, "Up", uploadBps);
}

void showNetdataError()
{
  if (!s_lvglInitialized)
  {
    return;
  }
  if (s_downloadLabel)
  {
    lv_label_set_text(s_downloadLabel, "Down: error");
  }
  if (s_uploadLabel)
  {
    lv_label_set_text(s_uploadLabel, "Up: error");
  }
}

void createRateScreen()
{
  if (!s_lvglInitialized)
  {
    return;
  }

  lv_obj_t *screen = lv_scr_act();
  s_downloadLabel = lv_label_create(screen);
  lv_label_set_text(s_downloadLabel, "Down: --");
  lv_obj_align(s_downloadLabel, LV_ALIGN_CENTER, 0, -16);

  s_uploadLabel = lv_label_create(screen);
  lv_label_set_text(s_uploadLabel, "Up: --");
  lv_obj_align(s_uploadLabel, LV_ALIGN_CENTER, 0, 16);
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

  if (s_lvglInitialized)
  {
    lv_timer_handler();
  }
  delay(5);
}
