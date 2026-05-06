#pragma once
#include <Arduino.h>
#include "states.h"
#include "config.h"
#include "servo_eye.h"

// ═══════════════════════════════════════════════════════════════════════════
//  behaviour.h — eye state machine
//
//  Consumes DetectionResult from detectionQueue.
//  Drives eye via setGazeDeg() + setArousal() + blink().
//
//  Graceful degradation stack:
//    Level 4: flock coordination (future WiFi)
//    Level 3: camera detection → TRACKING / FOCUS
//    Level 2: saliency above threshold → arousal increase
//    Level 1: autonomous idle (SCANNING/DOZING/SLEEPING) — always runs
//
//  The state machine never halts. If vision fails, it falls to Level 1.
// ═══════════════════════════════════════════════════════════════════════════

class Behaviour {
public:
    void begin();

    // Call every loop tick from Core 1
    // Reads latest DetectionResult from queue (non-blocking)
    void update();

    EyeState getState() const { return _state; }

private:
    EyeState _state     = EyeState::SCANNING;
    uint32_t _stateMs   = 0;   // millis() when we entered current state
    uint32_t _lastBlink = 0;

    // Last known target (for lingering after loss)
    float    _lastNormX  = 0.5f;
    float    _lastNormY  = 0.5f;
    float    _lastConf   = 0.0f;
    uint32_t _lastDetect = 0;

    // Scan oscillator
    float    _scanT = 0.0f;    // accumulates in seconds

    void _setState(EyeState s);
    void _doScanning(uint32_t now);
    void _doIdle(uint32_t now);
    void _doWaking(uint32_t now);
    void _doTracking(uint32_t now, const DetectionResult& det);
    void _doFocus(uint32_t now, const DetectionResult& det);
    void _doDozing(uint32_t now);
    void _doSleeping(uint32_t now);
    void _handleBlink(uint32_t now);

    // Arousal targets per state
    float _arousalForState(EyeState s) const;
};

extern Behaviour behaviour_sm;   // named _sm to avoid clash with BehaviourConfig