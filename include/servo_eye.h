#pragma once
#include <Arduino.h>
#include "config.h"
#include "m5servo8.h"

// ═══════════════════════════════════════════════════════════════════════════
//  servo_eye.h — smooth animated eye abstraction
//
//  Sits on top of M5Servo8. Provides:
//    setGazeTarget(normX, normY)  — 0.0–1.0 normalised target coords
//    setLids(fraction)            — 0.0 = closed, 1.0 = fully open
//    blink()                      — trigger a single non-blocking blink
//    setSleeping(bool)            — closes lids + relaxes gaze
//    update()                     — MUST be called every loop tick
//
//  All motion is smoothed via EMA — no jerky steps, no blocking delays.
//  The alpha (smoothing speed) is set externally so the behaviour layer
//  can vary it per state (lazy idle vs. snappy tracking).
// ═══════════════════════════════════════════════════════════════════════════

class ServoEye {
public:
    // ── One-time setup ───────────────────────────────────────────────────────
    // Call after M5Servo8::begin(). Moves to neutral / closed position.
    bool begin();

    // ── Gaze control ────────────────────────────────────────────────────────
    // normX/Y: 0.0–1.0 (camera frame coordinates, auto-mapped to servo range)
    // alpha:   EMA smoothing factor (0.0=frozen, 1.0=instant) — vary per state
    void setGazeTarget(float normX, float normY, float alpha = 0.1f);

    // Direct degree control (bypasses normalised mapping — useful for scanning)
    void setGazeDeg(float panDeg, float tiltDeg, float alpha = 0.1f);

    // ── Eyelid control ───────────────────────────────────────────────────────
    // fraction: 0.0 = fully closed, 1.0 = fully open, 0.5 = half/drowsy
    void setLids(float fraction, float alpha = 0.08f);

    // ── Blink ────────────────────────────────────────────────────────────────
    // Non-blocking. If a blink is already in progress it is ignored.
    void blink();

    // ── Convenience state setters ────────────────────────────────────────────
    void setSleeping();    // lids closed, gaze drifts to neutral
    void setDozing();      // lids half-open, slow drift
    void setAwake();       // lids fully open

    // ── Update — call every loop iteration ───────────────────────────────────
    // Advances EMA smoothing and writes to servos.
    // Returns true if any servo value actually changed this tick.
    bool update();

    // ── Accessors (for web status page) ─────────────────────────────────────
    float getPanDeg()    const { return _curPan;  }
    float getTiltDeg()   const { return _curTilt; }
    float getLidFrac()   const { return _curLid;  }
    bool  isBlinking()   const { return _blinking; }

private:
    // Current (smoothed) positions
    float _curPan   = PAN_CENTER;
    float _curTilt  = TILT_CENTER;
    float _curLid   = 0.0f;          // start closed

    // Targets (set by caller, chased by EMA in update())
    float _tgtPan   = PAN_CENTER;
    float _tgtTilt  = TILT_CENTER;
    float _tgtLid   = 0.0f;

    // Per-axis EMA alphas
    float _alphaGaze = 0.1f;
    float _alphaLid  = 0.08f;

    // Last values actually written to hardware (for dead-zone check)
    float _lastPan  = -999.f;
    float _lastTilt = -999.f;
    float _lastLid  = -999.f;

    // Blink state machine
    bool     _blinking    = false;
    uint32_t _blinkStart  = 0;
    float    _lidBeforeBlink = 0.0f;  // restore lid fraction after blink

    // ── Internal helpers ─────────────────────────────────────────────────────
    // Map normalised 0–1 camera coords to servo degrees
    float _normToPan(float n)  const;
    float _normToTilt(float n) const;

    // Convert lid fraction to top/bottom servo degrees
    void  _lidToDeg(float frac, uint8_t& topDeg, uint8_t& botDeg) const;

    // Write to hardware only when value has changed enough (saves I2C traffic)
    void  _writeIfChanged(uint8_t ch, float deg, float& lastWritten,
                          float threshold = 0.6f);
};

extern ServoEye eye;
