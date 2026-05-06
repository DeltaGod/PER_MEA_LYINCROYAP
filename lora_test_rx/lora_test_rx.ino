/*
  LoRa Link Test — RECEIVER
  T-Beam V1.1, 433 MHz
  Listens for packets and prints them with RSSI and SNR.
  Upload this to the RX T-Beam, lora_test_tx to the other.
*/

#include <SPI.h>
#include <LoRa.h>

#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   (-1)
#define LORA_IRQ   26
#define LORA_BAND  433E6

static uint32_t rxCount  = 0;
static uint32_t lastMs   = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== LoRa RX Test ===");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (LoRa.begin(LORA_BAND) != 1) {
    Serial.println("[LORA] ERROR: init failed");
    for (;;);
  }
  Serial.println("[LORA] OK — 433 MHz, listening…");
  Serial.println("---");
}

void loop() {
  int pktSize = LoRa.parsePacket();
  if (pktSize > 0) {
    String data = LoRa.readString();
    int    rssi = LoRa.packetRssi();
    float  snr  = LoRa.packetSnr();
    rxCount++;

    Serial.print("[RX #");
    Serial.print(rxCount);
    Serial.print("]  rssi=");
    Serial.print(rssi);
    Serial.print(" dBm  snr=");
    Serial.print(snr, 1);
    Serial.print(" dB  \"");
    Serial.print(data);
    Serial.println("\"");

    lastMs = millis();
  }

  // Warn if nothing received for > 5 s
  if (lastMs > 0 && (millis() - lastMs) > 5000) {
    Serial.println("[WARN] no packet for 5 s — check TX or antenna");
    lastMs = millis();
  }
}
