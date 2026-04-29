#pragma once
#include <Arduino.h>

namespace BoardConfig {

// I2C — AXP192 power manager (T-Beam internal bus)
// GPIO21/22 are reclaimed as RC inputs after AXP192 init + Wire.end()
static constexpr uint8_t I2C_SDA_PIN = 21;
static constexpr uint8_t I2C_SCL_PIN = 22;

// RC receiver inputs — interrupt-driven PWM (Pro-Tronik R8X)
static constexpr uint8_t RX_CH2_PIN = 13;  // sail toggle
static constexpr uint8_t RX_CH3_PIN = 22;  // propulsion throttle  (shared I2C SCL — OK after Wire.end)
static constexpr uint8_t RX_CH4_PIN = 21;  // rotor / differential  (shared I2C SDA — OK after Wire.end)
static constexpr uint8_t RX_CH5_PIN =  4;  // mode selector
static constexpr uint8_t RX_CH6_PIN = 23;  // reserved — WARNING: T-Beam V1.1 connects LoRa RST to GPIO23
                                            // Verify PCB schematic before enabling CH6 interrupt

// Actuator PWM outputs — MCPWM only, never LEDC (LEDC breaks RC interrupts)
static constexpr uint8_t SAIL_SERVO_PIN  =  2;
static constexpr uint8_t ROTOR_SERVO_PIN = 25;
static constexpr uint8_t ESC1_PIN        = 32;  // also T-Beam LoRa DIO1 — OK while LoRa unused
static constexpr uint8_t ESC2_PIN        = 33;  // also T-Beam LoRa DIO2 — OK while LoRa unused

// Battery ADC — external voltage divider (R1=560kΩ, R2=120kΩ)
// GPIO35 CANNOT be used: T-Beam V1.1 has its own low-impedance (~165Ω) battery
// monitoring divider permanently soldered to it, which clamps any external signal.
// GPIO36/GPIO39 (SVP/SVN) are not broken out as physical header pins on T-Beam V1.1.
// GPIO15 (ADC2_CH3) is the only free ADC pin on the header. ADC2 is fine here
// because this project never activates WiFi (LoRa only).
static constexpr uint8_t BATTERY_ADC_PIN = 15;

// GPS UART — Serial1 (future use)
static constexpr uint8_t  GPS_RX_PIN    = 34;   // input-only GPIO
static constexpr uint8_t  GPS_TX_PIN    = 12;
static constexpr uint32_t GPS_BAUD_RATE = 9600;

// LoRa SPI (future use)
static constexpr uint8_t LORA_SCK_PIN  =  5;
static constexpr uint8_t LORA_MISO_PIN = 19;
static constexpr uint8_t LORA_MOSI_PIN = 27;
static constexpr uint8_t LORA_CS_PIN   = 18;
static constexpr uint8_t LORA_RST_PIN  = 23;  // T-Beam V1.1 standard — verify vs Post_GPT GPIO14
static constexpr uint8_t LORA_IRQ_PIN  = 26;

// RC signal validation
static constexpr uint16_t RC_VALID_MIN_US = 800;
static constexpr uint16_t RC_VALID_MAX_US = 2200;
static constexpr uint32_t RC_TIMEOUT_US   = 100000;  // 100 ms without pulse = signal lost

// RC command range (Pro-Tronik PTR-6A)
static constexpr uint16_t RC_MIN_US     = 1000;
static constexpr uint16_t RC_MID_US     = 1500;
static constexpr uint16_t RC_MAX_US     = 2000;
static constexpr uint16_t RC_DEADBAND_US = 35;

// Mode selector thresholds — CH5 three-position switch
// <1000 → Automatic | 1400–1600 → ManualServo | >1800 → ManualProp
static constexpr uint16_t CH5_AUTO_THRESHOLD      = 1000;  // below → Automatic
static constexpr uint16_t CH5_SAIL_LOW_US         = 1400;  // ManualServo band low
static constexpr uint16_t CH5_SAIL_HIGH_US        = 1600;  // ManualServo band high
static constexpr uint16_t CH5_PROP_THRESHOLD      = 1800;  // above → ManualProp

} // namespace BoardConfig
