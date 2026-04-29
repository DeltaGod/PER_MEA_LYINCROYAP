#include "GpsUart.h"
#include "../config/BoardConfig.h"

void GpsUart::begin() {
    Serial1.begin(BoardConfig::GPS_BAUD_RATE, SERIAL_8N1,
                  BoardConfig::GPS_RX_PIN, BoardConfig::GPS_TX_PIN);
}

void GpsUart::update() {
    while (Serial1.available()) {
        gps_.encode(Serial1.read());
    }

    pos_.valid      = gps_.location.isValid();
    pos_.lat        = pos_.valid ? gps_.location.lat()           : 0.0;
    pos_.lon        = pos_.valid ? gps_.location.lng()           : 0.0;
    pos_.speedKmph  = gps_.speed.isValid()      ? (float)gps_.speed.kmph()    : 0.0f;
    pos_.courseDeg  = gps_.course.isValid()     ? (float)gps_.course.deg()    : 0.0f;
    pos_.satellites = gps_.satellites.isValid() ? (uint8_t)gps_.satellites.value() : 0;
    pos_.hdop       = gps_.hdop.isValid()       ? (float)gps_.hdop.hdop()     : 99.9f;
    pos_.ageMs      = gps_.location.isValid()   ? gps_.location.age()         : 0xFFFFFFFFUL;
}
