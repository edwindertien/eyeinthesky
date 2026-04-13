#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  m5servo8.h — thin I2C driver for the M5Stack 8Servo Unit (STM32F030F4)
//
//  Protocol summary (I2C addr 0x25):
//    Write angle:       reg = M5S8_REG_ANGLE_BASE + channel (0–7)
//                       data = angle byte (0–180)
//
//    Write pulse width: reg = M5S8_REG_PW_BASE + channel*2
//                       data = two bytes, little-endian (µs, 500–2500)
//
//    Motor power:       reg = M5S8_REG_MOTOR_POWER, data = 0x01 (on) / 0x00 (off)
//
//    Read current:      reg = M5S8_REG_CURRENT_L (two bytes, little-endian, mA)
//
//  Both angle-mode and pulse-width-mode work — we expose both.
//  Angle mode is convenient; PW mode is more precise for tuning end-stops.
// ═══════════════════════════════════════════════════════════════════════════

class M5Servo8 {
public:
    // ── Initialise I2C and verify the unit is present ────────────────────────
    // Returns true on success.
    bool begin(uint8_t sda = I2C_SDA, uint8_t scl = I2C_SCL,
               uint32_t freq = I2C_FREQ) {
        Wire.begin(sda, scl, freq);
        // Probe the device
        Wire.beginTransmission(M5SERVO8_ADDR);
        uint8_t err = Wire.endTransmission();
        if (err != 0) {
            Serial.printf("[M5Servo8] Device not found at 0x%02X (err %d)\n",
                          M5SERVO8_ADDR, err);
            return false;
        }
        Serial.printf("[M5Servo8] Found at 0x%02X\n", M5SERVO8_ADDR);
        return true;
    }

    // ── Servo power rail (MOS switch) ────────────────────────────────────────
    // Call powerOn() before any servo movement.
    // powerOff() releases all servos (no hold torque) — useful for sleep state.
    void powerOn()  { _writeReg(M5S8_REG_MOTOR_POWER, 0x01); _powered = true;  }
    void powerOff() { _writeReg(M5S8_REG_MOTOR_POWER, 0x00); _powered = false; }
    bool isPowered() const { return _powered; }

    // ── Set servo angle (0–180 degrees) ─────────────────────────────────────
    // channel: 0–7 (matching SERVO_CH_* constants in config.h)
    void setAngle(uint8_t channel, uint8_t angle) {
        if (channel > 7) return;
        angle = constrain(angle, 0, 180);
        _writeReg(M5S8_REG_ANGLE_BASE + channel, angle);
    }

    // ── Set servo pulse width in microseconds (500–2500µs) ───────────────────
    // More precise than angle mode; use for calibrating end-stops.
    void setPulseWidth(uint8_t channel, uint16_t us) {
        if (channel > 7) return;
        us = constrain(us, 500, 2500);
        uint8_t reg = M5S8_REG_PW_BASE + channel * 2;
        Wire.beginTransmission(M5SERVO8_ADDR);
        Wire.write(reg);
        Wire.write(us & 0xFF);          // low byte
        Wire.write((us >> 8) & 0xFF);   // high byte
        Wire.endTransmission();
    }

    // ── Read total servo rail current (milliamps) ─────────────────────────────
    // Useful for detecting stall / overload in the web dashboard
    uint16_t readCurrentMA() {
        Wire.beginTransmission(M5SERVO8_ADDR);
        Wire.write(M5S8_REG_CURRENT_L);
        Wire.endTransmission(false);    // repeated start
        Wire.requestFrom((uint8_t)M5SERVO8_ADDR, (uint8_t)2);
        if (Wire.available() < 2) return 0;
        uint16_t lo = Wire.read();
        uint16_t hi = Wire.read();
        return (hi << 8) | lo;
    }

    // ── Enable/disable individual servo channels ──────────────────────────────
    // bitmask: bit 0 = ch1, bit 7 = ch8. 0xFF = all enabled (default after powerOn)
    void setChannelEnable(uint8_t bitmask) {
        _writeReg(M5S8_REG_SERVO_ENABLE, bitmask);
    }

private:
    bool _powered = false;

    void _writeReg(uint8_t reg, uint8_t value) {
        Wire.beginTransmission(M5SERVO8_ADDR);
        Wire.write(reg);
        Wire.write(value);
        Wire.endTransmission();
    }
};

// Singleton — included by servo_eye.cpp, declared extern elsewhere as needed
extern M5Servo8 servoBoard;