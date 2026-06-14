#include "behaviour.h"
#include "vision.h"
#include "blob_tracker.h"
#include "config.h"
#include "calibration.h"
#include <cmath>

Behaviour behaviour_sm;

// Timing constants
static const uint32_t STARTLE_MS        = 500;   // startle flash duration
static const uint32_t SCARED_MIN_MS     = 3000;  // minimum time in SCARED
static const uint32_t SCAN_TO_DOZE_MS   = 10000; // scan -> doze if no blobs
static const uint32_t DOZE_TO_SLEEP_MS  = 20000; // doze -> sleep

// ── begin ─────────────────────────────────────────────────────────────────────
void Behaviour::begin() {
    _scanT          = 0.0f;
    _lastBlinkMs    = millis();
    _startleMs      = 0;
    _wakeResetMs    = 0;
    _wakeResetDone  = false;
    _targetBlobId   = 0;
    _dwellStartMs   = millis();
    _lastMotionMs   = 0;
    _hasCommitted   = false;
    _isFirstBoot    = true;

    // Always start SLEEPING -- background model needs to build first
    _setState(EyeState::SLEEPING);
    Serial.println("[Behaviour] Started in SLEEPING -- waiting for background");
}

void Behaviour::_setState(EyeState s) {
    if (s != _state)
        Serial.printf("[State] %s -> %s\n", stateName(_state), stateName(s));
    _state   = s;
    _stateMs = millis();
}

// ── Arousal targets ───────────────────────────────────────────────────────────
float Behaviour::_arousalForState(EyeState s) const {
    switch (s) {
        case EyeState::SLEEPING:  return 0.0f;
        case EyeState::SCARED:    return 0.0f;   // lids shut
        case EyeState::DOZING:    return 0.2f;
        case EyeState::WAKING:    return 0.4f;
        case EyeState::SCANNING:  return 0.5f;
        case EyeState::IDLE:      return 0.5f;
        case EyeState::TRACKING:  return 0.55f;
        case EyeState::FOCUS:     return 0.5f;
        default:                   return 0.5f;
    }
}

// ── Blink (arousal-scaled) ────────────────────────────────────────────────────
void Behaviour::_handleBlink(uint32_t now) {
    if (_state == EyeState::SLEEPING ||
        _state == EyeState::SCARED)   return;

    float arousal  = eye.getArousal();
    float scale    = 1.0f + (1.0f - arousal) * 3.0f;
    uint32_t interval = (uint32_t)(behaviour.blinkIntervalMs * scale);

    int32_t jitter = (int32_t)random(-(int32_t)behaviour.blinkJitterMs,
                                      (int32_t)behaviour.blinkJitterMs);
    if (now - _lastBlinkMs > (uint32_t)max(500L, (long)interval + jitter)) {
        // Drowsy blink is slower
        uint32_t saved = behaviour.blinkDurationMs;
        behaviour.blinkDurationMs = (uint32_t)(saved * (1.0f + (1.0f-arousal)*2.0f));
        eye.blink();
        behaviour.blinkDurationMs = saved;
        _lastBlinkMs = now;
    }
}

// ── _dwellExpired ─────────────────────────────────────────────────────────────
bool Behaviour::_dwellExpired(uint32_t now) const {
    float dx   = _gazeX - 0.5f;
    float dy   = _gazeY - 0.5f;
    float dist = sqrtf(dx*dx + dy*dy) * 1.41f;
    uint32_t dwell = (uint32_t)(800.0f + dist * 1200.0f);
    return (now - _dwellStartMs) > dwell;
}

// ── _pickNextBlob ─────────────────────────────────────────────────────────────
int Behaviour::_pickNextBlob(const BlobResult& blobs) const {
    if (blobs.count == 0) return -1;
    if (blobs.count == 1) return 0;

    int currentIdx = -1;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].id == _targetBlobId) { currentIdx = i; break; }

    int nextIdx = (currentIdx + 1) % blobs.count;
    float dx = blobs.blobs[nextIdx].normX - _gazeX;
    float dy = blobs.blobs[nextIdx].normY - _gazeY;
    if (sqrtf(dx*dx + dy*dy) < 0.12f && blobs.count > 2)
        nextIdx = (nextIdx + 1) % blobs.count;
    return nextIdx;
}

// ── State handlers ────────────────────────────────────────────────────────────

void Behaviour::_doSleeping(uint32_t now, const BlobResult& blobs) {
    eye.setSleeping();

    // Wait for background model to be ready before waking at all
    if (!visionBgSettled) return;



    // First boot: wake once after bg settled so eye opens and scans
    if (_isFirstBoot) {
        _isFirstBoot   = false;
        _startleMs     = 0;
        _wakeResetMs   = now;
        _wakeResetDone = false;
        Serial.println("[Sleep->Wake] FIRST BOOT");
        _setState(EyeState::WAKING);
        return;
    }

    // After that: only wake on substantial confirmed blob (score > 0.15)
    bool realMotion = false;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].score > 0.15f) { realMotion = true; break; }

    if (realMotion) {
        scheduleBackgroundReset(1500);
        _wakeResetMs   = now;
        _wakeResetDone = false;
        _startleMs     = now;
        Serial.printf("[Sleep->Wake] realMotion blobs=%d score=%.2f\n",
                      blobs.count, blobs.count > 0 ? blobs.blobs[0].score : 0.0f);
        _setState(EyeState::WAKING);
    }
}

void Behaviour::_doWaking(uint32_t now) {
    eye.setGazeDeg(calibration.data.panCenter, calibration.data.tiltCenter, 0.03f);
    eye.setArousal(_arousalForState(EyeState::WAKING), 0.025f);

    uint32_t elapsed = now - _stateMs;

    // At 1.5s: lids mostly open, scheduled bgreset fires automatically
    // At 3.5s: check if background has settled before allowing tracking
    if (elapsed > 3500 && visionBgSettled) {
        _setState(EyeState::SCANNING);
    }
}

void Behaviour::_doScanning(uint32_t now, const BlobResult& blobs) {
    _scanT += 0.020f;
    float pan  = calibration.data.panCenter
               + sinf(_scanT * 0.28f) * behaviour.scanPanAmp
               + sinf(_scanT * 0.61f) * behaviour.scanPanAmp * 0.25f;
    float tilt = calibration.data.tiltCenter
               + sinf(_scanT * 0.19f) * behaviour.scanTiltAmp;
    eye.setGazeDeg(pan, tilt, behaviour.gazeAlphaIdle);
    eye.setArousal(_arousalForState(EyeState::SCANNING), 0.04f);

    // Only start tracking if background is settled AND blobs are stable
    // Only track substantial blobs
    bool realBlob = false;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].score > 0.10f) { realBlob = true; break; }

    if (realBlob && visionBgSettled) {
        _startleMs    = now;
        _hasCommitted = false;
        _setState(EyeState::TRACKING);
    } else if (now - _stateMs > SCAN_TO_DOZE_MS) {
        _setState(EyeState::DOZING);
    }
}

void Behaviour::_doTracking(uint32_t now, const BlobResult& blobs) {
    // Startle flash on entry
    static const uint32_t SD = STARTLE_MS;
    if (_startleMs > 0 && (now - _startleMs) < SD) {
        float t = 1.0f - (float)(now - _startleMs) / SD;
        eye.setArousal(0.55f + t * 0.45f, 0.3f);
    } else {
        _startleMs = 0;
        eye.setArousal(_arousalForState(EyeState::TRACKING), 0.04f);
    }

    if (!blobs.anyMotion) {
        _setState(EyeState::SCANNING);
        return;
    }

    // Find current target blob
    int currentIdx = -1;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].id == _targetBlobId) { currentIdx = i; break; }

    // Shift to next blob when dwell expires or no current target
    bool shouldShift = _dwellExpired(now) || currentIdx < 0;

    if (shouldShift) {
        int nextIdx = _pickNextBlob(blobs);
        if (nextIdx >= 0) {
            const Blob& target = blobs.blobs[nextIdx];
            float dx   = target.normX - _gazeX;
            float dy   = target.normY - _gazeY;
            float dist = sqrtf(dx*dx + dy*dy);
            if (currentIdx < 0 || dist > 0.10f || _dwellExpired(now)) {
                Serial.printf("[Gaze] Blob%d  %.2f,%.2f -> %.2f,%.2f  dist=%.2f\n",
                              target.id, _gazeX, _gazeY,
                              target.normX, target.normY, dist);
                _gazeX        = target.normX;
                _gazeY        = target.normY;
                _targetBlobId = target.id;
                _dwellStartMs = now;
                _hasCommitted = true;
            }
        }
    }

    // Gaze with predictive lead
    float leadX = _gazeX, leadY = _gazeY;
    if (currentIdx >= 0) {
        const Blob& b = blobs.blobs[currentIdx];
        leadX = constrain(_gazeX + b.vx * 0.15f, 0.05f, 0.95f);
        leadY = constrain(_gazeY + b.vy * 0.15f, 0.05f, 0.95f);
    }
    eye.setGazeNorm(leadX, leadY, behaviour.gazeAlphaTrack);
    _lastMotionMs = now;

    if (_hasCommitted && (now - _stateMs) > 8000)
        _setState(EyeState::FOCUS);
}

void Behaviour::_doFocus(uint32_t now, const BlobResult& blobs) {
    eye.setArousal(_arousalForState(EyeState::FOCUS), 0.03f);
    if (!blobs.anyMotion) { _setState(EyeState::SCANNING); return; }

    // Find current blob
    int idx = -1;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].id == _targetBlobId) { idx = i; break; }
    if (idx < 0) { _setState(EyeState::TRACKING); return; }

    eye.setGazeNorm(blobs.blobs[idx].normX, blobs.blobs[idx].normY,
                    behaviour.gazeAlphaFocus);
    _lastMotionMs = now;

    // New blob appeared -> back to tracking
    if (blobs.count > 1) _setState(EyeState::TRACKING);
}

void Behaviour::_doDozing(uint32_t now, const BlobResult& blobs) {
    eye.setDozing();
    _scanT += 0.005f;
    float pan = calibration.data.panCenter
              + sinf(_scanT * 0.08f) * behaviour.scanPanAmp * 0.3f;
    eye.setGazeDeg(pan, calibration.data.tiltCenter, 0.015f);

    // Wake only on confirmed, substantial blobs -- not noise.
    // A real person creates blobs with score > 0.15.
    // Noise/artifacts typically score very low.
    bool realMotion = false;
    for (int i = 0; i < blobs.count; i++) {
        if (blobs.blobs[i].score > 0.15f) { realMotion = true; break; }
    }

    if (realMotion && visionBgSettled) {
        scheduleBackgroundReset(1500);
        _wakeResetMs   = now;
        _wakeResetDone = false;
        _startleMs     = now;
        _setState(EyeState::WAKING);
    } else if (now - _stateMs > DOZE_TO_SLEEP_MS) {
        _setState(EyeState::SLEEPING);
    }
}

void Behaviour::_doScared(uint32_t now, const BlobResult& blobs) {
    // Lids snap shut, eye centers
    eye.setArousal(0.0f, 0.5f);
    eye.setGazeDeg(calibration.data.panCenter, calibration.data.tiltCenter, 0.08f);

    uint32_t elapsed = now - _stateMs;

    // Phase 1 (0-500ms): hard flinch, don't even check blobs
    if (elapsed < 500) return;

    // Phase 2: hold closed until scene settled
    // Conditions: minimum time elapsed, bg settled, at most 1 blob
    bool settled = (elapsed > SCARED_MIN_MS) &&
                   visionBgSettled &&
                   (blobs.count <= 1);

    if (settled) {
        Serial.println("[Scared] Scene settled -- cautious reopen");
        scheduleBackgroundReset(500);  // one more clean reset before opening
        _wakeResetMs   = now;
        _wakeResetDone = false;
        _startleMs     = 0;            // no startle -- slow cautious open
        _setState(EyeState::WAKING);
    }
}

// ── update ────────────────────────────────────────────────────────────────────
void Behaviour::update() {
    uint32_t now = millis();

    // Read latest blob result
    BlobResult blobs = {};
    if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);
    if (blobs.anyMotion) _lastMotionMs = now;

    // -- Shift detection (highest priority, from any state) -------------------
    if (blobTrackerShiftDetected) {
        blobTrackerShiftDetected = false;
        Serial.println("[Behaviour] Shift! -> SCARED");
        // Schedule bg reset to fire once lids have closed (500ms)
        scheduleBackgroundReset(500);
        _wakeResetMs   = now;
        _wakeResetDone = false;
        _setState(EyeState::SCARED);
        eye.setArousal(0.0f, 0.5f);
        _handleBlink(now);
        return;
    }

    switch (_state) {
        case EyeState::SLEEPING:  _doSleeping(now, blobs);   break;
        case EyeState::WAKING:    _doWaking(now);             break;
        case EyeState::SCANNING:  _doScanning(now, blobs);   break;
        case EyeState::TRACKING:  _doTracking(now, blobs);   break;
        case EyeState::FOCUS:     _doFocus(now, blobs);       break;
        case EyeState::DOZING:    _doDozing(now, blobs);      break;
        case EyeState::SCARED:    _doScared(now, blobs);      break;
        case EyeState::IDLE:      _setState(EyeState::SCANNING); break;
    }

    if (_state != EyeState::SLEEPING &&
        _state != EyeState::SCARED)
        _handleBlink(now);
}