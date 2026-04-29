#include "BatteryAdc.h"
#include "../config/BoardConfig.h"

// R1=560kΩ (high side), R2=120kΩ (low side to GND)
// V_bat = V_pin × (R1+R2)/R2
static constexpr float DIVIDER_RATIO = (560000.0f + 120000.0f) / 120000.0f;

void BatteryAdc::begin() {
    analogSetPinAttenuation(BoardConfig::BATTERY_ADC_PIN, ADC_11db);
}

float BatteryAdc::readVolts() {
    uint32_t vPin_mV = analogReadMilliVolts(BoardConfig::BATTERY_ADC_PIN);
    return (vPin_mV * DIVIDER_RATIO) / 1000.0f;
}
