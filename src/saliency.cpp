#include "saliency.h"
#include <cstring>
#include <cmath>

SaliencyMap saliency;

// ── begin ─────────────────────────────────────────────────────────────────────
bool SaliencyMap::begin() {
    size_t mapSize = SAL_W * SAL_H;

    // All working maps go in PSRAM
    _bgModel   = (uint8_t*)ps_malloc(mapSize);
    _prevGrey  = (uint8_t*)ps_malloc(mapSize);
    _motionMap = (uint8_t*)ps_malloc(mapSize);
    _colorMap  = (uint8_t*)ps_malloc(mapSize);
    _brightMap = (uint8_t*)ps_malloc(mapSize);
    _salMap    = (uint8_t*)ps_malloc(mapSize);
    _salGrey   = (uint8_t*)ps_malloc(mapSize);
    _habitMap  = (float*)  ps_malloc(mapSize * sizeof(float));

    if (!_bgModel || !_prevGrey || !_motionMap || !_colorMap ||
        !_brightMap || !_salMap || !_salGrey || !_habitMap) {
        Serial.println("[Saliency] PSRAM allocation failed!");
        return false;
    }

    memset(_bgModel,   128, mapSize);  // neutral grey background
    memset(_prevGrey,  128, mapSize);
    memset(_motionMap,   0, mapSize);
    memset(_colorMap,    0, mapSize);
    memset(_brightMap,   0, mapSize);
    memset(_salMap,      0, mapSize);
    memset(_salGrey,   128, mapSize);

    for (size_t i = 0; i < mapSize; i++) _habitMap[i] = 0.0f;

    Serial.printf("[Saliency] %u bytes in PSRAM, maps %dx%d\n",
                  (unsigned)(mapSize * (7 + sizeof(float))), SAL_W, SAL_H);
    return true;
}

// ── _downsample ───────────────────────────────────────────────────────────────
// Downsample src (sw×sh) → dst (dw×dh) by averaging blocks
void SaliencyMap::_downsample(const uint8_t* src, int sw, int sh,
                               uint8_t* dst, int dw, int dh) {
    int bx = sw / dw, by = sh / dh;
    for (int dy = 0; dy < dh; dy++) {
        for (int dx = 0; dx < dw; dx++) {
            uint32_t sum = 0;
            int sx0 = dx*bx, sy0 = dy*by;
            for (int yy = 0; yy < by; yy++)
                for (int xx = 0; xx < bx; xx++)
                    sum += src[(sy0+yy)*sw + (sx0+xx)];
            dst[dy*dw + dx] = (uint8_t)(sum / (bx*by));
        }
    }
}

// ── _updateBackground ────────────────────────────────────────────────────────
// Slow exponential moving average background model
void SaliencyMap::_updateBackground(const uint8_t* grey) {
    // bgAlphaInt is learning rate × 1000 (e.g. 5 → 0.005)
    int alpha = bgAlphaInt;  // parts per 1000
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        int bg  = _bgModel[i];
        int cur = grey[i];
        // bg = bg + alpha*(cur-bg)/1000
        _bgModel[i] = (uint8_t)(bg + (alpha * (cur - bg)) / 1000);
    }
}

// ── _computeMotion ────────────────────────────────────────────────────────────
// Absolute difference from background model
void SaliencyMap::_computeMotion(const uint8_t* grey) {
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        int diff = (int)grey[i] - (int)_bgModel[i];
        if (diff < 0) diff = -diff;
        // Threshold small differences (sensor noise)
        diff = (diff > 12) ? diff : 0;
        _motionMap[i] = (uint8_t)(diff > 255 ? 255 : diff);
    }
    // Simple spatial max-pool 3×3 to fill small gaps
    // (avoid allocating scratch — do in-place with prev row trick)
    for (int y = 1; y < SAL_H-1; y++) {
        for (int x = 1; x < SAL_W-1; x++) {
            uint8_t m = 0;
            for (int dy = -1; dy <= 1; dy++)
                for (int dx = -1; dx <= 1; dx++) {
                    uint8_t v = _motionMap[(y+dy)*SAL_W + (x+dx)];
                    if (v > m) m = v;
                }
            _salGrey[y*SAL_W+x] = m;  // reuse _salGrey as temp
        }
    }
    // Copy dilated result back
    memcpy(_motionMap, _salGrey, SAL_W * SAL_H);
}

// ── _computeColorSaliency ────────────────────────────────────────────────────
// In greyscale we approximate colour saliency by measuring local
// contrast relative to the regional mean — regions that deviate from
// their surroundings are "colourful" (or at least visually distinct).
// True colour would need RGB/YUV input; this is a good greyscale proxy.
void SaliencyMap::_computeColorSaliency(const uint8_t* grey) {
    // Compute global mean
    uint32_t sum = 0;
    for (int i = 0; i < SAL_W * SAL_H; i++) sum += grey[i];
    int globalMean = (int)(sum / (SAL_W * SAL_H));

    // Local 5×5 mean contrast
    for (int y = 0; y < SAL_H; y++) {
        for (int x = 0; x < SAL_W; x++) {
            // Local mean in 5×5 window
            int localSum = 0, count = 0;
            for (int dy = -2; dy <= 2; dy++) {
                int ny = y + dy;
                if (ny < 0 || ny >= SAL_H) continue;
                for (int dx = -2; dx <= 2; dx++) {
                    int nx = x + dx;
                    if (nx < 0 || nx >= SAL_W) continue;
                    localSum += grey[ny*SAL_W + nx];
                    count++;
                }
            }
            int localMean = localSum / count;
            // Saliency = deviation from global mean (centre-surround)
            int diff = abs(localMean - globalMean);
            _colorMap[y*SAL_W + x] = (uint8_t)(diff > 255 ? 255 : diff);
        }
    }
}

// ── _computeBrightness ────────────────────────────────────────────────────────
// Centre-surround: bright spots on dark bg, or dark spots on bright bg
void SaliencyMap::_computeBrightness(const uint8_t* grey) {
    // Global mean
    uint32_t sum = 0;
    for (int i = 0; i < SAL_W * SAL_H; i++) sum += grey[i];
    int mean = (int)(sum / (SAL_W * SAL_H));

    for (int i = 0; i < SAL_W * SAL_H; i++) {
        int diff = abs((int)grey[i] - mean);
        _brightMap[i] = (uint8_t)(diff > 255 ? 255 : diff);
    }
}

// ── _decayHabit ───────────────────────────────────────────────────────────────
void SaliencyMap::_decayHabit() {
    float decay = habitDecay;
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        _habitMap[i] *= decay;
        if (_habitMap[i] < 0.001f) _habitMap[i] = 0.0f;
    }
}

// ── habituate ────────────────────────────────────────────────────────────────
// Stamp habituation at normalised position — call when eye is tracking
void SaliencyMap::habituate(float normX, float normY) {
    int cx = (int)(normX * SAL_W);
    int cy = (int)(normY * SAL_H);
    cx = cx < 0 ? 0 : (cx >= SAL_W ? SAL_W-1 : cx);
    cy = cy < 0 ? 0 : (cy >= SAL_H ? SAL_H-1 : cy);

    // Gaussian stamp: 5×5 around fixation point
    for (int dy = -4; dy <= 4; dy++) {
        int ny = cy + dy;
        if (ny < 0 || ny >= SAL_H) continue;
        for (int dx = -4; dx <= 4; dx++) {
            int nx = cx + dx;
            if (nx < 0 || nx >= SAL_W) continue;
            float g = expf(-(dx*dx + dy*dy) / 8.0f);  // sigma≈2
            _habitMap[ny*SAL_W + nx] += habitStrength * g;
            if (_habitMap[ny*SAL_W + nx] > 1.0f)
                _habitMap[ny*SAL_W + nx] = 1.0f;
        }
    }
}

// ── _combineMaps ─────────────────────────────────────────────────────────────
void SaliencyMap::_combineMaps() {
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        float s = wMotion * (_motionMap[i]  / 255.0f)
                + wColor  * (_colorMap[i]   / 255.0f)
                + wBright * (_brightMap[i]  / 255.0f)
                - wHabit  * _habitMap[i];
        if (s < 0.0f) s = 0.0f;
        if (s > 1.0f) s = 1.0f;
        _salMap[i] = (uint8_t)(s * 255.0f);
    }
}

// ── _findPeak ────────────────────────────────────────────────────────────────
// Find centroid of top-N saliency pixels (more stable than pure peak)
SaliencyResult SaliencyMap::_findPeak() {
    SaliencyResult r = {0.5f, 0.5f, 0.0f, false};

    // Find max value
    uint8_t maxVal = 0;
    for (int i = 0; i < SAL_W * SAL_H; i++)
        if (_salMap[i] > maxVal) maxVal = _salMap[i];

    if (maxVal < (uint8_t)(threshold * 255.0f)) return r;  // nothing salient

    // Centroid of pixels above 70% of max
    uint8_t cutoff = (uint8_t)(maxVal * 0.7f);
    float sumX = 0, sumY = 0, sumW = 0;
    for (int y = 0; y < SAL_H; y++) {
        for (int x = 0; x < SAL_W; x++) {
            uint8_t v = _salMap[y*SAL_W + x];
            if (v >= cutoff) {
                float w = v / 255.0f;
                sumX += x * w;
                sumY += y * w;
                sumW += w;
            }
        }
    }

    if (sumW < 0.001f) return r;

    r.normX      = (sumX / sumW) / SAL_W;
    r.normY      = (sumY / sumW) / SAL_H;
    r.confidence = maxVal / 255.0f;
    r.valid      = true;
    return r;
}

// ── process ───────────────────────────────────────────────────────────────────
SaliencyResult SaliencyMap::process(const uint8_t* grey, int w, int h) {
    // Downsample full frame to working resolution
    _downsample(grey, w, h, _salGrey, SAL_W, SAL_H);

    // Background model update
    if (!_bgInit) {
        memcpy(_bgModel, _salGrey, SAL_W * SAL_H);
        memcpy(_prevGrey, _salGrey, SAL_W * SAL_H);
        _bgInit = true;
    }
    _updateBackground(_salGrey);

    // Feature channels
    _computeMotion(_salGrey);
    _computeColorSaliency(_salGrey);
    _computeBrightness(_salGrey);

    // Habituation decay
    _decayHabit();

    // Combine into final saliency map
    _combineMaps();

    // Find winner
    SaliencyResult result = _findPeak();

    memcpy(_prevGrey, _salGrey, SAL_W * SAL_H);
    return result;
}