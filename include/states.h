#pragma once
#include <Arduino.h>

// ===========================================================================
//  states.h -- eye state machine types
//
//  State diagram:
//
//   SLEEPING --(motion/peer)---> WAKING ---> SCANNING
//      ▲                                      │
//      │                                 (detection)
//   DOZING ◄--(dozeToSleep)-- IDLE ◄------TRACKING
//                                  ◄------  FOCUS
// ===========================================================================

enum class EyeState : uint8_t {
    SLEEPING = 0,   // Lids fully closed, micro-tremor only
    WAKING,         // Lids opening, drifting to neutral
    SCANNING,       // Lazy sinusoidal sweep -- Level 1 baseline, always beautiful
    IDLE,           // Near-neutral, blinks, subtle drift
    TRACKING,       // Actively following a detected target
    FOCUS,          // Sustained lock -- intense, minimal motion
    DOZING,         // Half-closed lids, slow drift toward sleep
    SCARED,         // Camera shifted -- lids snap shut, wait for scene to settle
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
        case EyeState::SCARED:    return "SCARED";
        default:                   return "UNKNOWN";
    }
}

// -- Detection result passed from vision -> behaviour ---------------------------
struct DetectionResult {
    bool     valid;         // True if there is a salient target
    float    normX;         // 0.0 (left) … 1.0 (right) in camera frame
    float    normY;         // 0.0 (top)  … 1.0 (bottom)
    float    confidence;    // 0.0-1.0 -- strength of detection
    uint32_t timestamp;     // millis() at time of detection
};

// -- Peer hint broadcast over UDP between units --------------------------------
struct __attribute__((packed)) PeerHint {
    uint8_t  unitId;        // Sending unit ID (1-255)
    uint8_t  state;         // EyeState cast to uint8_t
    float    normX;         // Detected target normalised position
    float    normY;
    float    confidence;
    uint32_t timestamp;     // millis() on sender -- used for timeout
};