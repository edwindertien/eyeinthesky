#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  m5servo8.h — Driver for M5Stack Unit 8Servos (STM32F030, 8× PWM)
//  I2C address: 0x25
//
//  Register map (verified against official M5_UNIT_8SERVO firmware):
//    0x00 + ch     pin mode  (0=digital_in, 1=digital_out, 2=adc,
//                             3=servo, 4=rgb, 5=pwm)
//    0x10 + ch     digital output value
//    0x20 + ch     digital input read
//    0x30 + ch     analog input 8-bit
//    0x40 + ch*2   analog input 12-bit (little-endian)
//    0x50 + ch     servo angle 0–180 degrees   ← main control register
//    0x60 + ch*2   servo pulse width µs, little-endian [lo, hi]
//    0x70 + ch*3   RGB LED (24-bit)
//    0x90 + ch     PWM 8-bit
//    0xA0          servo current (float, 4 bytes, amps)
//    0xFE          firmware version
//    0xFF          I2C address
//
//  CRITICAL: all pins default to DIGITAL_INPUT_MODE on power-up.
//  begin() calls setAllPinMode(SERVO_CTL_MODE) — without this no PWM appears.
// ═══════════════════════════════════════════════════════════════════════════

#include <Arduino.h>
#include <Wire.h>

typedef enum {
    DIGITAL_INPUT_MODE  = 0,
    DIGITAL_OUTPUT_MODE = 1,
    ADC_INPUT_MODE      = 2,
    SERVO_CTL_MODE      = 3,
    RGB_LED_MODE        = 4,
    PWM_MODE            = 5
} servo8_io_mode_t;

class M5Servo8 {
public:
    static constexpr uint8_t DEFAULT_ADDR = 0x25;
    static constexpr uint8_t NUM_CH       = 8;

    explicit M5Servo8(uint8_t addr = DEFAULT_ADDR, TwoWire* wire = &Wire)
        : _addr(addr), _wire(wire) {}

    // Call after Wire.begin(). Returns false if device not found.
    bool begin() {
        delay(10);
        _wire->beginTransmission(_addr);
        if (_wire->endTransmission() != 0) {
            Serial.printf("[M5Servo8] Not found at 0x%02X — check wiring\n", _addr);
            return false;
        }
        Serial.printf("[M5Servo8] Found at 0x%02X, firmware v%d\n",
                      _addr, getFirmwareVersion());

        // CRITICAL — must set servo mode before any angle writes have effect
        setAllPinMode(SERVO_CTL_MODE);
        return true;
    }

    // Set all 8 channels to the same pin mode (burst write)
    bool setAllPinMode(servo8_io_mode_t mode) {
        uint8_t data[8];
        memset(data, (uint8_t)mode, 8);
        return _writeBytes(0x00, data, 8);
    }

    // Set a single channel pin mode
    bool setOnePinMode(uint8_t ch, servo8_io_mode_t mode) {
        if (ch >= NUM_CH) return false;
        uint8_t data = (uint8_t)mode;
        return _writeBytes(0x00 + ch, &data, 1);
    }

    // Set servo angle (0–180 degrees)
    bool setServoAngle(uint8_t ch, uint8_t angle) {
        if (ch >= NUM_CH) return false;
        if (angle > 180) angle = 180;
        return _writeBytes(0x50 + ch, &angle, 1);
    }

    // Set all channels to the same angle
    void setAllAngles(uint8_t angle) {
        for (uint8_t ch = 0; ch < NUM_CH; ch++) setServoAngle(ch, angle);
    }

    // Set servo pulse width directly in microseconds (500–2500 µs)
    // More precise than angle mode — use for end-stop calibration
    bool setServoPulse(uint8_t ch, uint16_t us) {
        if (ch >= NUM_CH) return false;
        uint8_t data[2] = { (uint8_t)(us & 0xFF),
                            (uint8_t)((us >> 8) & 0xFF) };
        return _writeBytes(0x60 + ch * 2, data, 2);
    }

    // Read total servo rail current in milliamps
    // Register 0xA0 returns a 4-byte IEEE float in amps
    uint16_t readCurrentMA() {
        uint8_t buf[4] = {0};
        if (!_readBytes(0xA0, buf, 4)) return 0;
        float amps = 0.0f;
        memcpy(&amps, buf, 4);
        return (uint16_t)(amps * 1000.0f);
    }

    uint8_t getFirmwareVersion() {
        uint8_t v = 0;
        _readBytes(0xFE, &v, 1);
        return v;
    }

    bool isConnected() {
        _wire->beginTransmission(_addr);
        return _wire->endTransmission() == 0;
    }

private:
    uint8_t  _addr;
    TwoWire* _wire;

    bool _writeBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        for (uint8_t i = 0; i < len; i++) _wire->write(buf[i]);
        return _wire->endTransmission() == 0;
    }

    bool _readBytes(uint8_t reg, uint8_t* buf, uint8_t len) {
        _wire->beginTransmission(_addr);
        _wire->write(reg);
        _wire->endTransmission(false);   // repeated start — no STOP between write and read
        if (_wire->requestFrom(_addr, len) != len) return false;
        for (uint8_t i = 0; i < len; i++) buf[i] = _wire->read();
        return true;
    }
};

extern M5Servo8 servoBoard;