#include "AxpPower.h"
#include "../config/BoardConfig.h"
#include <Wire.h>
#include <axp20x.h>

static AXP20X_Class axp;

bool AxpPower::begin() {
    Wire.begin(BoardConfig::I2C_SDA_PIN, BoardConfig::I2C_SCL_PIN);
    Wire.setClock(400000);

    if (axp.begin(Wire, AXP192_SLAVE_ADDRESS) != AXP_PASS) {
        Wire.end();
        return false;
    }

    // LoRa SX1276 — LDO2 at 3.3 V
    axp.setLDO2Voltage(3300);
    axp.setPowerOutPut(AXP192_LDO2, AXP202_ON);

    // GPS NEO-6M — LDO3 at 3.3 V
    axp.setLDO3Voltage(3300);
    axp.setPowerOutPut(AXP192_LDO3, AXP202_ON);

    // 3.3 V board rail — DCDC1
    axp.setDCDC1Voltage(3300);
    axp.setPowerOutPut(AXP192_DCDC1, AXP202_ON);

    // Enable battery voltage ADC for future monitoring
    axp.adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1, true);

    // Release I2C peripheral so GPIO21/22 become available for RC interrupts.
    // Wire.end() is available in ESP32 Arduino core 2.x+.
    Wire.end();
    return true;
}
