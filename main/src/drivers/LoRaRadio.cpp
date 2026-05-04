#include "LoRaRadio.h"
#include "../config/BoardConfig.h"
#include <SPI.h>
#include <LoRa.h>

bool LoRaRadio::begin() {
    SPI.begin(BoardConfig::LORA_SCK_PIN,
              BoardConfig::LORA_MISO_PIN,
              BoardConfig::LORA_MOSI_PIN,
              BoardConfig::LORA_CS_PIN);
    LoRa.setSPI(SPI);
    // RST = -1: skip hardware reset — GPIO23 is shared with RC CH6 on this PCB
    LoRa.setPins(BoardConfig::LORA_CS_PIN, -1, BoardConfig::LORA_IRQ_PIN);
    ready_ = (LoRa.begin(433E6) == 1);
    return ready_;
}

bool LoRaRadio::send(const char* msg) {
    if (!ready_) return false;
    LoRa.beginPacket();
    LoRa.print(msg);
    LoRa.endPacket();
    return true;
}

bool LoRaRadio::poll(char* buf, size_t maxLen) {
    if (!ready_) return false;
    int sz = LoRa.parsePacket();
    if (sz <= 0) return false;
    size_t i = 0;
    while (LoRa.available() && i < maxLen - 1) {
        buf[i++] = (char)LoRa.read();
    }
    buf[i]    = '\0';
    lastRssi_ = LoRa.packetRssi();
    return i > 0;
}
