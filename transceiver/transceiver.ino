/*
  SeaDrone Ground Station — Post-Claude
  ======================================
  Runs on a second LilyGO T-Beam V1.1 connected to a PC via USB.
  Bridges USB serial (115200 baud) <-> LoRa 433 MHz.

  RX: prints drone heartbeat JSON + RSSI to serial
  TX: accepts shorthand commands or raw JSON, wraps them and sends via LoRa

  Commands (type in serial monitor, send with Enter):
    navigate                → start autonomous mission
    stop                    → stop mission
    restart                 → reboot the drone
    wind-obs                → start wind observation (Phase 5)
    wind <0-359>            → set wind direction manually (degrees)
    home <lat,lon>          → set home / return point
    wpt <lat,lon,lat,lon,…> → load waypoints in Linear mode
    {…raw JSON…}            → pass through unchanged — use for scripting

  Note on wpt: LoRa packets are max 255 bytes.  With the JSON wrapper (~80 chars)
  you have ~170 chars for coordinates — roughly 8 waypoints at full precision.
  Use fewer decimal places to fit more waypoints.
*/

#include <SPI.h>
#include <LoRa.h>

// T-Beam V1.1 LoRa SPI pins
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_IRQ   26
// RST: GPIO23 on T-Beam V1.1.  Change to 23 if this board is used only as
// transceiver (no RC receiver on GPIO23).  Leave -1 to skip hardware reset.
#define LORA_RST   (-1)
#define LORA_BAND  433E6

// ---------- forward declarations ----------
static void sendCmd(const char* json);
static void handleInput(String& line);

// ==========================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== SeaDrone Ground Station ===");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (LoRa.begin(LORA_BAND) != 1) {
    Serial.println("[LORA] ERROR: init failed — check wiring / power");
    for (;;);
  }
  Serial.println("[LORA] OK — 433 MHz, listening…");
  Serial.println("Commands: navigate | stop | restart | wind-obs | wind <deg>");
  Serial.println("          home <lat,lon> | wpt <lat,lon,lat,lon,…> | {raw JSON}");
  Serial.println("---");
}

// ==========================================
void loop() {
  // --- RX: LoRa packet → Serial ---
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String data = LoRa.readString();
    Serial.print("[RX rssi=");
    Serial.print(LoRa.packetRssi());
    Serial.print("] ");
    Serial.println(data);
  }

  // --- TX: Serial line → LoRa ---
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;
    handleInput(line);
  }
}

// ==========================================
// Send a null-terminated JSON string via LoRa and echo it to serial
static void sendCmd(const char* json) {
  LoRa.beginPacket();
  LoRa.print(json);
  LoRa.endPacket();
  Serial.print("[TX] ");
  Serial.println(json);
}

// ==========================================
// Parse a line from serial and build the corresponding command JSON
static void handleInput(String& line) {

  // Raw JSON pass-through (for scripting / Python server)
  if (line.startsWith("{")) {
    char buf[256];
    line.toCharArray(buf, sizeof(buf));
    sendCmd(buf);
    return;
  }

  // --- navigate ---
  if (line.equalsIgnoreCase("navigate")) {
    sendCmd("{\"origin\":\"server\",\"type\":\"command\",\"message\":{\"navigate\":true}}");

  // --- stop ---
  } else if (line.equalsIgnoreCase("stop")) {
    sendCmd("{\"origin\":\"server\",\"type\":\"command\",\"message\":{\"stop\":true}}");

  // --- restart ---
  } else if (line.equalsIgnoreCase("restart")) {
    sendCmd("{\"origin\":\"server\",\"type\":\"command\",\"message\":{\"restart\":true}}");

  // --- wind-obs / wind-observation ---
  } else if (line.equalsIgnoreCase("wind-obs") ||
             line.equalsIgnoreCase("wind-observation")) {
    sendCmd("{\"origin\":\"server\",\"type\":\"command\",\"message\":{\"wind-observation\":true}}");

  // --- wind <deg> ---
  } else if (line.startsWith("wind ") || line.startsWith("wind\t")) {
    int deg = line.substring(5).toInt();
    if (deg < 0 || deg > 359) {
      Serial.println("[ERR] wind: value must be 0-359");
      return;
    }
    char buf[96];
    snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"wind-command\":{\"value\":%d}}}",
      deg);
    sendCmd(buf);

  // --- home <lat,lon> ---
  } else if (line.startsWith("home ") || line.startsWith("home\t")) {
    String args = line.substring(5);
    args.trim();
    int comma = args.indexOf(',');
    if (comma < 0) {
      Serial.println("[ERR] home: format is  home <lat,lon>");
      return;
    }
    double lat = args.substring(0, comma).toDouble();
    double lon = args.substring(comma + 1).toDouble();
    char buf[128];
    snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"home\":{\"lat\":%.6f,\"lon\":%.6f}}}",
      lat, lon);
    sendCmd(buf);

  // --- wpt <lat,lon,lat,lon,...> ---
  } else if (line.startsWith("wpt ") || line.startsWith("wpt\t")) {
    String pts = line.substring(4);
    pts.trim();
    if (pts.length() == 0) {
      Serial.println("[ERR] wpt: no coordinates given");
      return;
    }
    // Count waypoints: N pairs → 2N values → 2N-1 commas → N = (commas+1)/2
    int commas = 0;
    for (size_t i = 0; i < pts.length(); i++) if (pts[i] == ',') commas++;
    int n = (commas + 1) / 2;
    if (n < 1) {
      Serial.println("[ERR] wpt: need at least one lat,lon pair");
      return;
    }
    char buf[256];
    int written = snprintf(buf, sizeof(buf),
      "{\"origin\":\"server\",\"type\":\"command\","
      "\"message\":{\"waypoints\":{\"number\":%d,\"points\":\"%s\"}}}",
      n, pts.c_str());
    if (written >= (int)sizeof(buf)) {
      Serial.println("[ERR] wpt: command too long for LoRa packet (max ~8 waypoints at full precision)");
      return;
    }
    sendCmd(buf);

  } else {
    Serial.print("[ERR] unknown command: ");
    Serial.println(line);
    Serial.println("      try: navigate | stop | restart | wind <deg> | home <lat,lon> | wpt <lat,lon,...>");
  }
}
