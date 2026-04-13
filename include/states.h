#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
//  states.h — eye state machine types
// ═══════════════════════════════════════════════════════════════════════════

enum class EyeState : uint8_t {
    SLEEPING = 0,  // Lids fully closed, micro-tremor only
    WAKING,        // Lids opening, slow pan toward neutral
    SCANNING,      // Lazy sinusoidal pan/tilt, lids open — Level 1 baseline
    IDLE,          // Near-neutral gaze, occasional blink, subtle drift
    TRACKING,      // Actively following detected target
    FOCUS,         // Sustained lock on target — still, intense
    DOZING,        // Lids half-closed, slow drift — transitioning to sleep
};

inline const char* stateName(EyeState s) {
    switch (s) {
        case EyeState::SLEEPING:  return "SLEEPING";
        case EyeState::WAKING:    return "WAKING";
        case EyeState::SCANNING:  return "SCANNING";
        case EyeState::IDLE:      return "IDLE";
        case EyeState::TRACKING:  return "TRACKING";
        case EyeState::FOCUS:     return "FOCUS";
        case EyeState::DOZING:    return "DOZING";
        default:                   return "UNKNOWN";
    }
}

// ── Detection result: vision task → behaviour task ───────────────────────────
struct DetectionResult {
    bool     valid;        // Is there a salient target?
    float    normX;        // 0.0 (left) … 1.0 (right) in camera frame
    float    normY;        // 0.0 (top)  … 1.0 (bottom)
    float    confidence;   // 0.0–1.0
    uint32_t timestamp;    // millis() at detection
};

// ── Peer hint: broadcast over UDP between units ──────────────────────────────
struct __attribute__((packed)) PeerHint {
    uint8_t  unitId;
    uint8_t  state;
    float    normX;
    float    normY;
    float    confidence;
    uint32_t timestamp;
};
// Note: sizeof(PeerHint) == 1+1+4+4+4+4 = 18 bytes when packed