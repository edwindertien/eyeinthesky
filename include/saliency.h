#pragma once
#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  saliency.h — multi-channel saliency with habituation + multi-peak attention
//
//  Pipeline per frame:
//    1. Downsample QVGA → 80×60 working resolution
//    2. Background subtraction → motion channel
//    3. Local contrast → colour/pattern channel
//    4. Centre-surround → brightness channel
//    5. Combine channels: rawSaliency = w_motion*M + w_color*C + w_bright*B
//    6. Attended saliency: S = rawSaliency * (1 - wHabit * habitMap)
//       Habituation suppresses regions the eye has already visited
//    7. Find up to MAX_PEAKS local maxima (non-maximum suppression)
//    8. Select winner (highest attended saliency)
//    9. Stamp habituation at winner → boredom builds up
//   10. When all peaks fall below threshold → valid=false → eye sleeps
//
//  Boredom / sleep dynamics:
//    - Static scene: background adapts, motion=0, habit builds → sleep
//    - New motion: fresh region with zero habituation → instant wake
//    - Stationary interesting object: habituates over ~habitDecay^N frames
//    - Multiple objects: eye cycles between them as each habituates
// ═══════════════════════════════════════════════════════════════════════════

#define SAL_W       80    // working saliency resolution
#define SAL_H       60
#define MAX_PEAKS    5    // max salient regions to track simultaneously

struct SaliencyPeak {
    float normX, normY;   // position (0–1)
    float score;          // attended saliency score (0–1)
    bool  valid;
};

struct SaliencyResult {
    float normX, normY;   // winning peak position
    float confidence;     // 0–1
    bool  valid;
    // All found peaks (for debug / future flock use)
    SaliencyPeak peaks[MAX_PEAKS];
    int   numPeaks;
};

class SaliencyMap {
public:
    bool begin();

    // Process a QVGA greyscale frame — returns winning attention target
    SaliencyResult process(const uint8_t* grey, int w, int h);

    // Stamp habituation at normalised position (called by behaviour on tracking)
    // strength multiplier: 1.0 = normal, 2.0 = double stamp (focus state)
    void habituate(float normX, float normY, float strength = 1.0f);

    // Force-reset habituation map (e.g. after long sleep)
    void resetHabit();

    // Debug map accessors (same-task only)
    const uint8_t* getMotionMap()   const { return _motionMap; }
    const uint8_t* getSaliencyMap() const { return _salMap;    }
    const float*   getHabitMap()    const { return _habitMap;  }

    // ── Tunable parameters (exposed via serial commands) ─────────────────────
    //
    // Design intent:
    //   - Static scene:  colour/brightness habituate quickly → sleep
    //   - Moving object: motion channel dominates → stays salient longer
    //   - Relaxed pace:  slow alpha, long dwell, gradual drain before shift
    //
    // Channel weights — motion only; colour/brightness off by default
    // Static texture (contrast, brightness) habituates via habit map.
    // Colour/bright re-enabled via serial if needed for specific setups.
    float  wMotion       = 0.90f;  // motion almost exclusively
    float  wColor        = 0.06f;  // very low — static contrast nearly ignored
    float  wBright       = 0.04f;  // minimal — nearly off

    // Habituation — aggressive enough to drain static scenes within ~15s
    // habitStrength × fps = drain rate. At 11fps: 0.08×11 = 0.88/s stamp rate
    // With decay 0.994, equilibrium habit = strength/(1-decay) ≈ 0.08/0.006 = 13
    // which clamps to 1.0, so a fixated point fully suppresses in ~8-10s
    float  wHabit        = 1.0f;   // full suppression when habit=1
    // At 11fps, 15 frames ≈ 1.4s to drain. Maths:
    // h[t] = S * (1-D^t)/(1-D) → for h=0.85 in 15 frames at D=0.97:
    // S = 0.85 * (1-0.97) / (1-0.97^15) = 0.85*0.03/0.366 ≈ 0.070
    float  habitStrength = 0.070f; // drains fixated point to 85% in ~1.4s @ 11fps
    float  habitDecay    = 0.970f; // faster decay — clears in ~10s after leaving

    // Detection
    float  threshold     = 0.30f;
    int    maxPeaks      = 3;      // hard limit on salient points reported
    int    peakRadius    = 10;     // larger radius — fewer, well-separated peaks
    int    bgAlphaInt    = 3;

private:
    uint8_t* _bgModel   = nullptr;
    uint8_t* _prevGrey  = nullptr;
    uint8_t* _motionMap = nullptr;
    uint8_t* _colorMap  = nullptr;
    uint8_t* _brightMap = nullptr;
    uint8_t* _salRaw    = nullptr;  // raw combined (before habituation)
    uint8_t* _salMap    = nullptr;  // attended saliency (after habituation)
    uint8_t* _salGrey   = nullptr;  // downsampled working grey
    float*   _habitMap  = nullptr;

    bool _bgInit = false;

    void _updateBackground(const uint8_t* grey);
    void _computeMotion(const uint8_t* grey);
    void _computeColorSaliency(const uint8_t* grey);
    void _computeBrightness(const uint8_t* grey);
    void _decayHabit();
    void _combineAndAttend();
    void _findPeaks(SaliencyResult& result);
    void _downsample(const uint8_t* src, int sw, int sh,
                     uint8_t* dst, int dw, int dh);
};

extern SaliencyMap saliency;