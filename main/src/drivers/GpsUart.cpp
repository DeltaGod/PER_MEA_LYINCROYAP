#include "GpsUart.h"
#include "../config/BoardConfig.h"
#include "../config/DebugConfig.h"

void GpsUart::begin() {
    gsvTotal_.begin(gps_, "GPGSV", 3);  // field 3 = total satellites in view
    Serial1.begin(BoardConfig::GPS_BAUD_RATE, SERIAL_8N1,
                  BoardConfig::GPS_RX_PIN, BoardConfig::GPS_TX_PIN);
    DBG("GPS", "Serial1 started %u baud RX=GPIO%d TX=GPIO%d",
        (unsigned)BoardConfig::GPS_BAUD_RATE,
        BoardConfig::GPS_RX_PIN, BoardConfig::GPS_TX_PIN);
}

uint8_t GpsUart::satsInView() {
    return gsvTotal_.isValid() ? (uint8_t)atoi(gsvTotal_.value()) : 0;
}

void GpsUart::update() {
    while (Serial1.available()) {
        char c = Serial1.read();
        gps_.encode(c);

        // Capture last complete NMEA sentence into completedLine_
        if (c == '\n') {
            lineBuf_[lineLen_] = '\0';
            memcpy(completedLine_, lineBuf_, lineLen_ + 1);
            lineLen_ = 0;
        } else if (c != '\r' && lineLen_ < sizeof(lineBuf_) - 1) {
            lineBuf_[lineLen_++] = c;
        }
    }

    const bool nowValid = gps_.location.isValid();

    // Log GPS fix transitions
    if (nowValid && !prevValid_) {
        DBG("GPS", "FIX ACQUIRED  lat=%.6f lon=%.6f sats=%u hdop=%.1f",
            gps_.location.lat(), gps_.location.lng(),
            gps_.satellites.isValid() ? (unsigned)gps_.satellites.value() : 0u,
            gps_.hdop.isValid() ? (float)gps_.hdop.hdop() : 99.9f);
    } else if (!nowValid && prevValid_) {
        DBG("GPS", "FIX LOST");
    }
    prevValid_ = nowValid;

    pos_.valid      = nowValid;
    pos_.lat        = nowValid ? gps_.location.lat()           : 0.0;
    pos_.lon        = nowValid ? gps_.location.lng()           : 0.0;
    pos_.speedKmph  = gps_.speed.isValid()      ? (float)gps_.speed.kmph()    : 0.0f;
    pos_.courseDeg  = gps_.course.isValid()     ? (float)gps_.course.deg()    : 0.0f;
    pos_.satellites = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    pos_.hdop       = gps_.hdop.isValid()       ? (float)gps_.hdop.hdop()     : 99.9f;
    pos_.ageMs      = gps_.location.isValid()   ? gps_.location.age()         : 0xFFFFFFFFUL;
}
