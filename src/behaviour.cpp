#include "behaviour.h"
#include "vision.h"
#include "config.h"
#include "calibration.h"
#include <cmath>

Behaviour behaviour_sm;

// Dwell times: how long to look at each blob before shifting
static const uint32_t DWELL_MIN_MS   = 800;   // minimum dwell (small movement)
static const uint32_t DWELL_MAX_MS   = 2000;  // maximum dwell (large movement)
static const uint32_t STARTLE_MS     = 500;   // startle duration
static const uint32_t SCAN_TO_DOZE   = 10000; // scanning → dozing if no blobs
static const uint32_t DOZE_TO_SLEEP  = 20000; // dozing → sleeping

// ── begin ─────────────────────────────────────────────────────────────────────
void Behaviour::begin() {
    _scanT       = 0.0f;
    _lastBlinkMs = millis();
    _startleMs   = 0;
    _targetBlobId = 0;
    _dwellStartMs = millis();
    _lastMotionMs = 0;
    _setState(EyeState::WAKING);
    Serial.println("[Behaviour] Started");
}

void Behaviour::_setState(EyeState s) {
    if (s != _state)
        Serial.printf("[State] %s → %s\n", stateName(_state), stateName(s));
    _state   = s;
    _stateMs = millis();
}

// ── Arousal targets ───────────────────────────────────────────────────────────
float Behaviour::_arousalForState(EyeState s) const {
    switch (s) {
        case EyeState::SLEEPING:  return 0.0f;
        case EyeState::DOZING:    return 0.2f;
        case EyeState::WAKING:    return 0.4f;
        case EyeState::SCANNING:  return 0.5f;
        case EyeState::TRACKING:  return 0.55f;
        case EyeState::FOCUS:     return 0.5f;
        default:                   return 0.5f;
    }
}

// ── Blink ─────────────────────────────────────────────────────────────────────
void Behaviour::_handleBlink(uint32_t now) {
    if (_state == EyeState::SLEEPING) return;

    // Blink interval scales inversely with arousal:
    //   arousal 1.0 (alert)  -> blinkIntervalMs base          (~5s)
    //   arousal 0.5 (normal) -> blinkIntervalMs * 1.5         (~7.5s)
    //   arousal 0.2 (dozing) -> blinkIntervalMs * 4           (~20s)
    //   arousal 0.0 (asleep) -> would be infinite, but we return above
    // Also: blink duration slows when drowsy (lazy, heavy blink)
    float arousal = eye.getArousal();
    float scale   = 1.0f + (1.0f - arousal) * 3.0f;  // 1x at full, 4x at zero
    uint32_t interval = (uint32_t)(behaviour.blinkIntervalMs * scale);

    int32_t jitter = (int32_t)random(-(int32_t)behaviour.blinkJitterMs,
                                      (int32_t)behaviour.blinkJitterMs);
    if (now - _lastBlinkMs > (uint32_t)max(500L, (long)interval + jitter)) {
        // Drowsy blink is slower — arousal 1.0 = normal speed, 0.2 = 3x slower
        // We temporarily override blinkDurationMs, then restore it
        uint32_t savedDur = behaviour.blinkDurationMs;
        behaviour.blinkDurationMs = (uint32_t)(savedDur * (1.0f + (1.0f - arousal) * 2.0f));
        eye.blink();
        behaviour.blinkDurationMs = savedDur;  // restore immediately after trigger
        _lastBlinkMs = now;
    }
}

// ── _dwellExpired ─────────────────────────────────────────────────────────────
bool Behaviour::_dwellExpired(uint32_t now) const {
    // Dwell scales with travel distance — far jumps linger longer
    float dx   = _gazeX - 0.5f;  // rough travel from center
    float dy   = _gazeY - 0.5f;
    float dist = sqrtf(dx*dx + dy*dy) * 1.41f;  // 0=center, 1=corner
    uint32_t dwell = (uint32_t)(DWELL_MIN_MS + dist * (DWELL_MAX_MS - DWELL_MIN_MS));
    return (now - _dwellStartMs) > dwell;
}

// ── _pickNextBlob ─────────────────────────────────────────────────────────────
// Round-robin with preference for large travel distance
int Behaviour::_pickNextBlob(const BlobResult& blobs) const {
    if (blobs.count == 0) return -1;
    if (blobs.count == 1) return 0;

    // Find blob currently being tracked
    int currentIdx = -1;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].id == _targetBlobId) { currentIdx = i; break; }

    // Try next blob in round-robin
    int nextIdx = (currentIdx + 1) % blobs.count;

    // If next blob is very close to current gaze, try the one after
    // (prefer large travel for artistically interesting gaze shifts)
    float dx = blobs.blobs[nextIdx].normX - _gazeX;
    float dy = blobs.blobs[nextIdx].normY - _gazeY;
    if (sqrtf(dx*dx + dy*dy) < 0.12f && blobs.count > 2) {
        nextIdx = (nextIdx + 1) % blobs.count;
    }
    return nextIdx;
}

// ── State handlers ────────────────────────────────────────────────────────────

void Behaviour::_doSleeping(uint32_t now, const BlobResult& blobs) {
    eye.setSleeping();
    if (blobs.anyMotion) {
        // Reset background so wake starts with clean slate.
        // _doWaking will wait for background to re-stabilise before tracking.
        blobTracker.resetBackground();
        _startleMs     = now;
        _wakeResetMs   = now;
        _wakeResetDone = false;  // will do a second reset once lids open
        _setState(EyeState::WAKING);
    }
}

void Behaviour::_doWaking(uint32_t now) {
    eye.setGazeDeg(calibration.data.panCenter, calibration.data.tiltCenter, 0.03f);
    eye.setArousal(_arousalForState(EyeState::WAKING), 0.025f);

    uint32_t elapsed = now - _stateMs;

    // At 1.5s into waking, lids are mostly open — issue a fresh bgreset
    // so the background is learned from the real current scene, not the
    // half-initialised state from the sleep→wake transition.
    if (elapsed > 1500 && !_wakeResetDone) {
        blobTracker.resetBackground();
        _wakeResetMs   = now;
        _wakeResetDone = true;
        Serial.println("[Wake] Background reset — lids open, learning scene");
    }

    // Wait 2s after the fresh reset before allowing tracking
    bool bgSettled = _wakeResetDone && (now - _wakeResetMs) > 2000;
    if (bgSettled) {
        _setState(EyeState::SCANNING);
    }
}

void Behaviour::_doScanning(uint32_t now, const BlobResult& blobs) {
    // Lazy sinusoidal scan
    _scanT += 0.020f;
    float pan  = calibration.data.panCenter
               + sinf(_scanT * 0.28f) * behaviour.scanPanAmp
               + sinf(_scanT * 0.61f) * behaviour.scanPanAmp * 0.25f;
    float tilt = calibration.data.tiltCenter
               + sinf(_scanT * 0.19f) * behaviour.scanTiltAmp;
    eye.setGazeDeg(pan, tilt, behaviour.gazeAlphaIdle);
    eye.setArousal(_arousalForState(EyeState::SCANNING), 0.04f);

    if (blobs.anyMotion) {
        _startleMs = now;
        _targetBlobId = 0;  // force fresh selection
        _setState(EyeState::TRACKING);
    } else if (now - _stateMs > SCAN_TO_DOZE) {
        _setState(EyeState::DOZING);
    }
}

void Behaviour::_doTracking(uint32_t now, const BlobResult& blobs) {
    // ── Startle: brief wide-eye on first detection ────────────────────────────
    if (_startleMs > 0 && (now - _startleMs) < STARTLE_MS) {
        float t = 1.0f - (float)(now - _startleMs) / STARTLE_MS;
        eye.setArousal(0.55f + t * 0.45f, 0.3f);
    } else {
        _startleMs = 0;
        eye.setArousal(_arousalForState(EyeState::TRACKING), 0.04f);
    }

    if (!blobs.anyMotion) {
        _setState(EyeState::SCANNING);
        return;
    }

    // ── Find current target blob in result ────────────────────────────────────
    int currentIdx = -1;
    for (int i = 0; i < blobs.count; i++)
        if (blobs.blobs[i].id == _targetBlobId) { currentIdx = i; break; }

    // ── Shift to next blob when dwell expires ────────────────────────────────
    bool shouldShift = _dwellExpired(now) || currentIdx < 0;

    if (shouldShift) {
        int nextIdx = _pickNextBlob(blobs);
        if (nextIdx >= 0) {
            const Blob& target = blobs.blobs[nextIdx];

            // Only shift if travel is meaningful (>10% of frame)
            float dx   = target.normX - _gazeX;
            float dy   = target.normY - _gazeY;
            float dist = sqrtf(dx*dx + dy*dy);

            if (currentIdx < 0 || dist > 0.10f || _dwellExpired(now)) {
                Serial.printf("[Gaze] Blob%d  %.2f,%.2f → %.2f,%.2f  dist=%.2f\n",
                              target.id, _gazeX, _gazeY,
                              target.normX, target.normY, dist);
                _gazeX        = target.normX;
                _gazeY        = target.normY;
                _targetBlobId = target.id;
                _dwellStartMs = now;
                _lastBlobIdx  = nextIdx;
            }
        }
    }

    // ── Gaze at committed position ────────────────────────────────────────────
    // Add slight predictive lead in direction of blob velocity
    float leadX = _gazeX, leadY = _gazeY;
    if (currentIdx >= 0) {
        const Blob& b = blobs.blobs[currentIdx];
        // Lead by ~0.15s of travel
        leadX = _gazeX + b.vx * 0.15f;
        leadY = _gazeY + b.vy * 0.15f;
        leadX = constrain(leadX, 0.05f, 0.95f);
        leadY = constrain(leadY, 0.05f, 0.95f);
    }
    eye.setGazeNorm(leadX, leadY, behaviour.gazeAlphaTrack);
    _lastMotionMs = now;
}

void Behaviour::_doDozing(uint32_t now, const BlobResult& blobs) {
    eye.setDozing();
    _scanT += 0.005f;
    float pan = calibration.data.panCenter
              + sinf(_scanT * 0.08f) * behaviour.scanPanAmp * 0.3f;
    eye.setGazeDeg(pan, calibration.data.tiltCenter, 0.015f);

    if (blobs.anyMotion) {
        _startleMs = now;
        _setState(EyeState::WAKING);
    } else if (now - _stateMs > DOZE_TO_SLEEP) {
        _setState(EyeState::SLEEPING);
    }
}

// ── update ────────────────────────────────────────────────────────────────────
void Behaviour::update() {
    uint32_t now = millis();

    // Read latest blob result (non-blocking)
    BlobResult blobs = {};
    if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);

    if (blobs.anyMotion) _lastMotionMs = now;

    switch (_state) {
        case EyeState::SLEEPING: _doSleeping(now, blobs);      break;
        case EyeState::WAKING:   _doWaking(now);               break;
        case EyeState::SCANNING: _doScanning(now, blobs);      break;
        case EyeState::TRACKING: _doTracking(now, blobs);      break;
        case EyeState::DOZING:   _doDozing(now, blobs);        break;
        case EyeState::IDLE:     _setState(EyeState::SCANNING); break;
        case EyeState::FOCUS:    _setState(EyeState::TRACKING); break;
    }

    if (_state != EyeState::SLEEPING) _handleBlink(now);
}