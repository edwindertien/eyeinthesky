#include <Arduino.h>
#include "config.h"
#include "states.h"
#include "m5servo8.h"
#include "servo_eye.h"

// ═══════════════════════════════════════════════════════════════════════════
//  main.cpp — Stage 1: servo foundation test
//
//  No camera, no WiFi, no state machine yet.
//  Goal: verify I2C comms, servo limits, eyelid geometry, and smooth motion.
//
//  What this does:
//    1. Wake sequence  — lids open slowly from closed
//    2. Scan sequence  — pan left/right, tilt up/down
//    3. Blink test     — random blinks during idle
//    4. Sleep sequence — lids close, return to neutral
//    5. Repeat
//
//  Serial output (115200) reports current servo positions and current draw.
//  Use this to calibrate PAN/TILT/LID constants in config.h.
// ═══════════════════════════════════════════════════════════════════════════

// Global config instances (extern declared in config.h)
SaliencyConfig  saliency;
BehaviourConfig behaviour;

// ── Test state machine ────────────────────────────────────────────────────────
enum class TestPhase {
    WAKING, SCAN_PAN, SCAN_TILT, CORNERS, IDLE_BLINK, SLEEPING
};

TestPhase   phase       = TestPhase::WAKING;
uint32_t    phaseStart  = 0;
uint32_t    lastBlink   = 0;
uint32_t    lastReport  = 0;

void setPhase(TestPhase p) {
    phase      = p;
    phaseStart = millis();
    Serial.printf("[Test] Phase → %d\n", (int)p);
}

// ── Arduino setup ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n\n=== EyeWatcher Stage 1 — Servo Test ===");

    // ── Init I2C + M5 8Servo unit ────────────────────────────────────────────
    if (!servoBoard.begin()) {
        Serial.println("[FATAL] M5Servo8 not found — check wiring and I2C address");
        // Don't halt — allow USB serial inspection. Blink user LED as error signal.
        pinMode(LED_BUILTIN, OUTPUT);
        while (true) {
            digitalWrite(LED_BUILTIN, LOW);  delay(200);  // XIAO LED active-low
            digitalWrite(LED_BUILTIN, HIGH); delay(200);
        }
    }

    // ── Init ServoEye (powers on servo rail, homes to neutral/closed) ────────
    eye.begin();

    // Brief pause to let servos reach start position before we start moving
    delay(800);

    setPhase(TestPhase::WAKING);
    lastBlink  = millis();
    lastReport = millis();

    Serial.println("[Setup] Ready — beginning test sequence");
    Serial.println("        Adjust PAN/TILT/LID constants in config.h as needed");
}

// ── Arduino loop ──────────────────────────────────────────────────────────────
void loop() {
    uint32_t now     = millis();
    uint32_t elapsed = now - phaseStart;

    // ── Phase sequencer ──────────────────────────────────────────────────────
    switch (phase) {

        // ── 1. Wake: open lids slowly over 1.5s ─────────────────────────────
        case TestPhase::WAKING:
            eye.setGazeDeg(PAN_CENTER, TILT_CENTER, 0.03f);
            eye.setLids(1.0f, 0.02f);         // slow open
            if (elapsed > 1500) setPhase(TestPhase::SCAN_PAN);
            break;

        // ── 2. Pan left and right ────────────────────────────────────────────
        case TestPhase::SCAN_PAN: {
            float t = (float)elapsed / 1000.0f;  // seconds
            float pan = PAN_CENTER + sinf(t * 0.8f) * (PAN_MAX - PAN_CENTER) * 0.8f;
            eye.setGazeDeg(pan, TILT_CENTER, 0.08f);
            eye.setLids(1.0f, 0.1f);
            if (elapsed > 6000) setPhase(TestPhase::SCAN_TILT);
            break;
        }

        // ── 3. Tilt up and down ──────────────────────────────────────────────
        case TestPhase::SCAN_TILT: {
            float t = (float)elapsed / 1000.0f;
            float tilt = TILT_CENTER + sinf(t * 0.7f) * (TILT_MAX - TILT_CENTER) * 0.7f;
            eye.setGazeDeg(PAN_CENTER, tilt, 0.08f);
            if (elapsed > 5000) setPhase(TestPhase::CORNERS);
            break;
        }

        // ── 4. Look at each corner in sequence ──────────────────────────────
        case TestPhase::CORNERS: {
            // Each corner held for 800ms
            static const float corners[4][2] = {
                {0.1f, 0.1f},   // top-left
                {0.9f, 0.1f},   // top-right
                {0.9f, 0.9f},   // bottom-right
                {0.1f, 0.9f},   // bottom-left
            };
            int idx = (elapsed / 900) % 4;
            eye.setGazeTarget(corners[idx][0], corners[idx][1], 0.12f);
            if (elapsed > 4000) setPhase(TestPhase::IDLE_BLINK);
            break;
        }

        // ── 5. Idle at neutral with random blinks ────────────────────────────
        case TestPhase::IDLE_BLINK: {
            // Subtle sinusoidal drift — this is the Level 1 baseline feel
            float t = (float)elapsed / 1000.0f;
            float pan  = PAN_CENTER  + sinf(t * 0.3f)  * 6.0f
                                     + sinf(t * 0.7f)  * 3.0f;
            float tilt = TILT_CENTER + sinf(t * 0.25f) * 4.0f;
            eye.setGazeDeg(pan, tilt, 0.04f);
            eye.setLids(1.0f, 0.06f);

            // Random blink every 3–6 seconds
            uint32_t blinkInterval = behaviour.blinkIntervalMs
                                   + (random(behaviour.blinkJitterMs * 2)
                                      - behaviour.blinkJitterMs);
            if (now - lastBlink > blinkInterval) {
                eye.blink();
                lastBlink = now;
            }

            if (elapsed > 8000) setPhase(TestPhase::SLEEPING);
            break;
        }

        // ── 6. Sleep: close lids, relax gaze ────────────────────────────────
        case TestPhase::SLEEPING:
            eye.setSleeping();
            if (elapsed > 3000) {
                // Loop back to wake
                setPhase(TestPhase::WAKING);
            }
            break;
    }

    // ── Update servo positions (EMA + write to hardware) ─────────────────────
    eye.update();

    // ── Periodic serial report (every 500ms) ─────────────────────────────────
    if (now - lastReport > 500) {
        lastReport = now;
        uint16_t mA = servoBoard.readCurrentMA();
        Serial.printf("[Eye] pan=%.1f tilt=%.1f lid=%.2f  current=%umA  phase=%d\n",
                      eye.getPanDeg(), eye.getTiltDeg(), eye.getLidFrac(),
                      mA, (int)phase);
    }

    // ── Loop cadence: ~50Hz is plenty for smooth SG90 motion ─────────────────
    // EMA smoothing means higher tick rate = smoother motion
    delay(20);
}
