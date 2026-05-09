#include "saliency.h"
#include <cstring>
#include <cmath>

SaliencyMap saliency;

// ── begin ─────────────────────────────────────────────────────────────────────
bool SaliencyMap::begin() {
    size_t sz = SAL_W * SAL_H;
    _bgModel   = (uint8_t*)ps_malloc(sz);
    _prevGrey  = (uint8_t*)ps_malloc(sz);
    _motionMap = (uint8_t*)ps_malloc(sz);
    _colorMap  = (uint8_t*)ps_malloc(sz);
    _brightMap = (uint8_t*)ps_malloc(sz);
    _salRaw    = (uint8_t*)ps_malloc(sz);
    _salMap    = (uint8_t*)ps_malloc(sz);
    _salGrey   = (uint8_t*)ps_malloc(sz);
    _habitMap  = (float*)  ps_malloc(sz * sizeof(float));

    if (!_bgModel || !_prevGrey || !_motionMap || !_colorMap ||
        !_brightMap || !_salRaw || !_salMap || !_salGrey || !_habitMap) {
        Serial.println("[Saliency] PSRAM alloc FAILED");
        return false;
    }
    memset(_bgModel,   128, sz);
    memset(_prevGrey,  128, sz);
    memset(_motionMap,   0, sz);
    memset(_colorMap,    0, sz);
    memset(_brightMap,   0, sz);
    memset(_salRaw,      0, sz);
    memset(_salMap,      0, sz);
    memset(_salGrey,   128, sz);
    for (size_t i = 0; i < sz; i++) _habitMap[i] = 0.0f;

    size_t total = sz * (8 + sizeof(float));
    Serial.printf("[Saliency] %u bytes in PSRAM, maps %dx%d, %d peaks\n",
                  (unsigned)total, SAL_W, SAL_H, MAX_PEAKS);
    return true;
}

// ── Downsample ────────────────────────────────────────────────────────────────
void SaliencyMap::_downsample(const uint8_t* src, int sw, int sh,
                               uint8_t* dst, int dw, int dh) {
    int bx = sw / dw, by = sh / dh;
    for (int dy = 0; dy < dh; dy++)
        for (int dx = 0; dx < dw; dx++) {
            uint32_t sum = 0;
            for (int yy = 0; yy < by; yy++)
                for (int xx = 0; xx < bx; xx++)
                    sum += src[(dy*by+yy)*sw + (dx*bx+xx)];
            dst[dy*dw+dx] = (uint8_t)(sum / (bx*by));
        }
}

// ── Background model (motion-gated) ───────────────────────────────────────────
// Only update background at pixels that are currently still.
// Moving pixels are NOT learned into the background — prevents fast-moving
// objects from being "absorbed" and stops static texture from dominating.
void SaliencyMap::_updateBackground(const uint8_t* grey) {
    int alpha = bgAlphaInt;
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        // Skip pixels that are currently moving
        if (_motionMap[i] > 20) continue;
        int bg = _bgModel[i];
        _bgModel[i] = (uint8_t)(bg + (alpha * ((int)grey[i] - bg)) / 1000);
    }
}

// ── Motion channel ────────────────────────────────────────────────────────────
void SaliencyMap::_computeMotion(const uint8_t* grey) {
    // Absolute difference from background + 3×3 dilation
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        int diff = abs((int)grey[i] - (int)_bgModel[i]);
        // Threshold at 18 — suppress JPEG/sensor noise (typically <15px diff)
        // Scale up genuine motion aggressively so it dominates
        _motionMap[i] = (diff > 18) ? (uint8_t)min(diff * 3, 255) : 0;
    }
    // Dilate into _salGrey as temp
    for (int y = 1; y < SAL_H-1; y++)
        for (int x = 1; x < SAL_W-1; x++) {
            uint8_t m = 0;
            for (int dy=-1; dy<=1; dy++)
                for (int dx=-1; dx<=1; dx++) {
                    uint8_t v = _motionMap[(y+dy)*SAL_W+(x+dx)];
                    if (v > m) m = v;
                }
            _salGrey[y*SAL_W+x] = m;
        }
    memcpy(_motionMap, _salGrey, SAL_W * SAL_H);
}

// ── Colour/contrast channel ───────────────────────────────────────────────────
void SaliencyMap::_computeColorSaliency(const uint8_t* grey) {
    // Global mean
    uint32_t sum = 0;
    for (int i = 0; i < SAL_W * SAL_H; i++) sum += grey[i];
    int gMean = (int)(sum / (SAL_W * SAL_H));

    // 5×5 local mean deviation from global mean
    for (int y = 0; y < SAL_H; y++)
        for (int x = 0; x < SAL_W; x++) {
            int lSum = 0, cnt = 0;
            for (int dy=-2; dy<=2; dy++) {
                int ny = y+dy; if (ny<0||ny>=SAL_H) continue;
                for (int dx=-2; dx<=2; dx++) {
                    int nx = x+dx; if (nx<0||nx>=SAL_W) continue;
                    lSum += grey[ny*SAL_W+nx]; cnt++;
                }
            }
            int diff = abs(lSum/cnt - gMean);
            // Threshold at 15 — only strong local contrast counts
            _colorMap[y*SAL_W+x] = (diff > 15) ? (uint8_t)min(diff * 2, 255) : 0;
        }
}

// ── Brightness centre-surround ────────────────────────────────────────────────
void SaliencyMap::_computeBrightness(const uint8_t* grey) {
    uint32_t sum = 0;
    for (int i = 0; i < SAL_W * SAL_H; i++) sum += grey[i];
    int mean = (int)(sum / (SAL_W * SAL_H));
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        int diff = abs((int)grey[i] - mean);
        // Threshold at 20 — only strong brightness deviations count
        _brightMap[i] = (diff > 20) ? (uint8_t)min(diff * 2, 255) : 0;
    }
}

// ── Decay habituation map ─────────────────────────────────────────────────────
void SaliencyMap::_decayHabit() {
    float d = habitDecay;
    for (int i = 0; i < SAL_W * SAL_H; i++) {
        _habitMap[i] *= d;
        if (_habitMap[i] < 0.001f) _habitMap[i] = 0.0f;
    }
}

// ── Habituate at normalised position ─────────────────────────────────────────
void SaliencyMap::habituate(float normX, float normY, float strength) {
    int cx = (int)constrain(normX * SAL_W, 0, SAL_W-1);
    int cy = (int)constrain(normY * SAL_H, 0, SAL_H-1);
    float s = habitStrength * strength;
    // Gaussian stamp, radius ~5 pixels (sigma≈2.5)
    for (int dy=-6; dy<=6; dy++) {
        int ny = cy+dy; if (ny<0||ny>=SAL_H) continue;
        for (int dx=-6; dx<=6; dx++) {
            int nx = cx+dx; if (nx<0||nx>=SAL_W) continue;
            float g = expf(-(dx*dx + dy*dy) / 12.5f);
            _habitMap[ny*SAL_W+nx] = fminf(1.0f, _habitMap[ny*SAL_W+nx] + s*g);
        }
    }
}

void SaliencyMap::resetHabit() {
    memset(_habitMap, 0, SAL_W * SAL_H * sizeof(float));
    Serial.println("[Saliency] Habit map reset");
}

// ── Combine channels + apply habituation suppression ─────────────────────────
void SaliencyMap::_combineAndAttend() {
    // Precompute peripheral boost map — edges/corners get a slight bonus.
    // This counteracts the tendency to fixate near frame center and encourages
    // the eye to make large, committed gaze shifts to the periphery.
    float cx = SAL_W * 0.5f;
    float cy = SAL_H * 0.5f;
    float maxDist = sqrtf(cx*cx + cy*cy);

    for (int y = 0; y < SAL_H; y++) {
        for (int x = 0; x < SAL_W; x++) {
            int i = y * SAL_W + x;

            // Raw saliency: weighted sum of channels
            float raw = wMotion * (_motionMap[i] / 255.0f)
                      + wColor  * (_colorMap[i]  / 255.0f)
                      + wBright * (_brightMap[i] / 255.0f);
            raw = fminf(1.0f, raw);

            // Peripheral boost: 0.0 at center, +0.15 at corners
            // Only applies if there's already some saliency (not pure noise boost)
            float dx = (x - cx) / cx;
            float dy = (y - cy) / cy;
            float periphery = sqrtf(dx*dx + dy*dy) * 0.5f;  // 0→~0.7
            if (raw > 0.05f) raw = fminf(1.0f, raw + periphery * 0.15f);

            _salRaw[i] = (uint8_t)(raw * 255.0f);

            // Attended saliency: suppressed by habituation
            float attended = raw * (1.0f - wHabit * _habitMap[i]);
            attended = fmaxf(0.0f, attended);
            _salMap[i] = (uint8_t)(attended * 255.0f);
        }
    }
}

// ── Find peaks using non-maximum suppression ──────────────────────────────────
void SaliencyMap::_findPeaks(SaliencyResult& result) {
    result.numPeaks = 0;
    result.valid    = false;
    result.normX = result.normY = 0.5f;
    result.confidence = 0.0f;

    // Make a copy of attended saliency map to suppress found peaks
    // (use _colorMap as scratch — it's rebuilt each frame)
    uint8_t* scratch = _colorMap;  // temporarily repurposed after _combineAndAttend
    memcpy(scratch, _salMap, SAL_W * SAL_H);

    uint8_t thr = (uint8_t)(threshold * 255.0f);

    int limit = min(maxPeaks, MAX_PEAKS);
    for (int p = 0; p < limit; p++) {
        // Find global maximum in scratch
        uint8_t maxVal = 0;
        int maxIdx = -1;
        for (int i = 0; i < SAL_W * SAL_H; i++) {
            if (scratch[i] > maxVal) { maxVal = scratch[i]; maxIdx = i; }
        }

        if (maxIdx < 0 || maxVal < thr) break;  // no more peaks above threshold

        int px = maxIdx % SAL_W;
        int py = maxIdx / SAL_W;

        // Use the raw peak pixel position as gaze target.
        // Centroid would pull toward center of spread-out regions — we want
        // the actual hotspot so the eye commits to a corner/edge position.
        // Small jitter correction: find sub-pixel peak in 3×3 window
        float sumX = 0, sumY = 0, sumW = 0;
        for (int dy = -1; dy <= 1; dy++) {
            int ny = py+dy; if (ny<0||ny>=SAL_H) continue;
            for (int dx = -1; dx <= 1; dx++) {
                int nx = px+dx; if (nx<0||nx>=SAL_W) continue;
                float w = scratch[ny*SAL_W+nx] / 255.0f;
                sumX += nx * w; sumY += ny * w; sumW += w;
            }
        }

        SaliencyPeak peak;
        // Use 3×3 sub-pixel refine — stays close to peak, not pulled to centroid
        peak.normX = (sumW > 0) ? (sumX/sumW) / SAL_W : (float)px/SAL_W;
        peak.normY = (sumW > 0) ? (sumY/sumW) / SAL_H : (float)py/SAL_H;
        peak.score = maxVal / 255.0f;
        peak.valid = true;
        result.peaks[result.numPeaks++] = peak;

        // Suppress this peak's region for NMS — full peakRadius
        for (int dy = -peakRadius; dy <= peakRadius; dy++) {
            int ny = py+dy; if (ny<0||ny>=SAL_H) continue;
            for (int dx = -peakRadius; dx <= peakRadius; dx++) {
                int nx = px+dx; if (nx<0||nx>=SAL_W) continue;
                scratch[ny*SAL_W+nx] = 0;
            }
        }
    }

    if (result.numPeaks > 0) {
        // Winner = first peak (highest attended saliency)
        result.normX      = result.peaks[0].normX;
        result.normY      = result.peaks[0].normY;
        result.confidence = result.peaks[0].score;
        result.valid      = true;
    }
}

// ── process ───────────────────────────────────────────────────────────────────
SaliencyResult SaliencyMap::process(const uint8_t* grey, int w, int h) {
    _downsample(grey, w, h, _salGrey, SAL_W, SAL_H);

    if (!_bgInit) {
        memcpy(_bgModel,  _salGrey, SAL_W * SAL_H);
        memcpy(_prevGrey, _salGrey, SAL_W * SAL_H);
        _bgInit = true;
    }

    // Motion FIRST — so background gate uses current frame's motion map
    _computeMotion(_salGrey);
    // Background updates only at non-moving pixels
    _updateBackground(_salGrey);
    _computeColorSaliency(_salGrey);
    _computeBrightness(_salGrey);
    _decayHabit();
    _combineAndAttend();

    SaliencyResult result;
    _findPeaks(result);

    memcpy(_prevGrey, _salGrey, SAL_W * SAL_H);
    return result;
}