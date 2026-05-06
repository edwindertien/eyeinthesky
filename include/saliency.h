#pragma once
#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  saliency.h — multi-channel saliency with habituation
//
//  Saliency score per pixel:
//    S = w_motion  * motion
//      + w_color   * colorSaliency
//      + w_bright  * brightnessSurround
//      - w_habit   * habitMap          ← suppresses held fixations
//
//  Habituation map:
//    A float map (in PSRAM) that accumulates where the eye has been looking.
//    Decays globally each frame. Creates the "boredom" / attention-shift
//    behaviour without any explicit state machine logic.
//
//  All maps are at WORKING resolution (QVGA/4 = 80×60) for CPU efficiency.
//  The thumbnail (DBG_W×DBG_H = 40×30) is derived from this.
// ═══════════════════════════════════════════════════════════════════════════

#define SAL_W   80    // working saliency resolution
#define SAL_H   60

struct SaliencyResult {
    float normX;        // 0.0–1.0 winning target position
    float normY;
    float confidence;   // 0.0–1.0 peak saliency score
    bool  valid;        // true if above threshold
};

class SaliencyMap {
public:
    // Allocate maps in PSRAM
    bool begin();

    // Feed a full QVGA (320×240) greyscale frame
    // Returns the winning target
    SaliencyResult process(const uint8_t* grey, int w, int h);

    // Stamp habituation at a normalised position (call when eye is tracking)
    void habituate(float normX, float normY);

    // Access maps for debug display (call from same task only)
    const uint8_t*  getMotionMap()   const { return _motionMap;   }
    const uint8_t*  getSaliencyMap() const { return _salMap;      }
    const float*    getHabitMap()    const { return _habitMap;     }

    // Tunable weights — all exposed via serial 'sal' command
    float  wMotion  = 0.6f;
    float  wColor   = 0.2f;
    float  wBright  = 0.2f;
    float  wHabit   = 0.5f;       // habituation suppression strength
    float  habitStrength = 0.08f; // how fast boredom builds per frame
    float  habitDecay    = 0.995f;// per-frame global decay (1.0=no decay)
    float  threshold     = 0.15f; // minimum saliency to report valid target
    int    bgAlphaInt    = 5;     // background learning rate × 1000

private:
    // Working maps — allocated in PSRAM
    uint8_t* _bgModel   = nullptr;  // background model (greyscale, SAL_W×SAL_H)
    uint8_t* _prevGrey  = nullptr;  // previous frame
    uint8_t* _motionMap = nullptr;  // motion channel (0–255)
    uint8_t* _salMap    = nullptr;  // combined saliency (0–255)
    float*   _habitMap  = nullptr;  // habituation (0.0–1.0, SAL_W×SAL_H)

    bool _bgInit = false;

    // Per-channel processing
    void _updateBackground(const uint8_t* grey);
    void _computeMotion(const uint8_t* grey);
    void _computeColorSaliency(const uint8_t* grey);
    void _computeBrightness(const uint8_t* grey);
    void _decayHabit();
    void _combineMaps();
    SaliencyResult _findPeak();

    // Downsample QVGA→SAL (factor 4×4)
    void _downsample(const uint8_t* src, int sw, int sh,
                     uint8_t* dst, int dw, int dh);

    // Scratch buffers (PSRAM)
    uint8_t* _colorMap  = nullptr;
    uint8_t* _brightMap = nullptr;
    uint8_t* _salGrey   = nullptr;  // downsampled working grey
};

extern SaliencyMap saliency;