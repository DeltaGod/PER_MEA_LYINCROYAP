#include "LoRaRadio.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"
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
    // First attempt — may fail if SX1276 is in a stuck TX state from a previous run
    ready_ = (LoRa.begin(433E6) == 1);
    if (!ready_) {
        // Force SX1276 out of stuck TX, then retry
        LoRa.sleep();
        delay(50);
        ready_ = (LoRa.begin(433E6) == 1);
        if (!ready_) {
            delay(100);
            LoRa.sleep();
            delay(50);
            ready_ = (LoRa.begin(433E6) == 1);
        }
    }
    DBG("Radio", "begin: %s (SCK=%d MISO=%d MOSI=%d CS=%d IRQ=%d)",
        ready_ ? "OK 433MHz" : "FAIL",
        BoardConfig::LORA_SCK_PIN, BoardConfig::LORA_MISO_PIN,
        BoardConfig::LORA_MOSI_PIN, BoardConfig::LORA_CS_PIN,
        BoardConfig::LORA_IRQ_PIN);
    return ready_;
}

bool LoRaRadio::send(const char* msg) {
    if (!ready_) return false;
    if (!LoRa.beginPacket()) {
        DBG("Radio", "TX skipped — still busy");
        return false;
    }
    DBG("Radio", "TX %u bytes", (unsigned)strlen(msg));
    LoRa.print(msg);
    // DIAGNOSTIC TEST: blocking TX (original approach that worked in Phase 3)
    LoRa.endPacket(false);
    return true;
}

bool LoRaRadio::poll(char* buf, size_t maxLen) {
    if (!ready_) return false;
    static uint32_t pollCount = 0;
    if (++pollCount % 100 == 0) {
        DBG("Radio", "DIAG #%lu op=0x%02X irq=0x%02X",
            (unsigned long)pollCount,
            (unsigned)LoRa.debugOpMode(),
            (unsigned)LoRa.debugIrqFlags());
    }
    int sz = LoRa.parsePacket();
    if (sz > 0) {
        rxDetected_++;
        DBG("Radio", "RX pkt detected sz=%d rssi=%d", sz, LoRa.packetRssi());
    }
    if (sz <= 0) return false;
    size_t i = 0;
    while (LoRa.available() && i < maxLen - 1) {
        buf[i++] = (char)LoRa.read();
    }
    buf[i]    = '\0';
    lastRssi_ = LoRa.packetRssi();
    DBG("Radio", "RX read %u bytes", (unsigned)i);
    return i > 0;
}
