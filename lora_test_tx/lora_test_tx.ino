/*
  LoRa Link Test — TRANSMITTER
  T-Beam V1.1, 433 MHz
  Sends "PING #N" every second and prints it to serial.
  Upload this to the TX T-Beam, lora_test_rx to the other.
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

static uint32_t counter = 0;

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("=== LoRa TX Test ===");

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (LoRa.begin(LORA_BAND) != 1) {
    Serial.println("[LORA] ERROR: init failed");
    for (;;);
  }
  Serial.println("[LORA] OK — 433 MHz, transmitting every 1 s");
  Serial.println("---");
}

void loop() {
  char msg[32];
  snprintf(msg, sizeof(msg), "PING #%lu", (unsigned long)counter++);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();

  Serial.print("[TX] ");
  Serial.println(msg);

  delay(1000);
}
