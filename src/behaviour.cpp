#include "behaviour.h"
#include "saliency.h"
#include "vision.h"
#include "config.h"

Behaviour behaviour_sm;

// ── begin ─────────────────────────────────────────────────────────────────────
void Behaviour::begin() {
    // Safety check — if BehaviourConfig is zero-initialised, set safe defaults
    if (behaviour.blinkDurationMs == 0) behaviour.blinkDurationMs = 110;
    if (behaviour.blinkIntervalMs == 0) behaviour.blinkIntervalMs = 3500;
    if (behaviour.blinkJitterMs   == 0) behaviour.blinkJitterMs   = 1500;
    if (behaviour.idleToDozeMs    == 0) behaviour.idleToDozeMs    = 12000;
    if (behaviour.dozeToSleepMs   == 0) behaviour.dozeToSleepMs   = 40000;
    if (behaviour.wakeDelayMs     == 0) behaviour.wakeDelayMs     = 300;
    if (behaviour.scanPanAmp      == 0) behaviour.scanPanAmp      = 18.0f;
    if (behaviour.scanTiltAmp     == 0) behaviour.scanTiltAmp     = 6.0f;
    if (behaviour.scanPeriodMs    == 0) behaviour.scanPeriodMs    = 9000.0f;
    if (behaviour.gazeAlphaIdle   == 0) behaviour.gazeAlphaIdle   = 0.04f;
    if (behaviour.gazeAlphaTrack  == 0) behaviour.gazeAlphaTrack  = 0.14f;
    if (behaviour.gazeAlphaFocus  == 0) behaviour.gazeAlphaFocus  = 0.06f;

    _scanT     = 0.0f;
    _lastBlink = millis();
    _setState(EyeState::WAKING);
    Serial.printf("[Behaviour] Started — idleToDoze=%lums dozeToSleep=%lums\n",
                  behaviour.idleToDozeMs, behaviour.dozeToSleepMs);
}

// ── _setState ────────────────────────────────────────────────────────────────
void Behaviour::_setState(EyeState s) {
    if (s != _state) {
        Serial.printf("[State] %s → %s\n", stateName(_state), stateName(s));
    }
    _state   = s;
    _stateMs = millis();
}

// ── _arousalForState ─────────────────────────────────────────────────────────
float Behaviour::_arousalForState(EyeState s) const {
    switch (s) {
        case EyeState::SLEEPING:  return 0.0f;
        case EyeState::DOZING:    return 0.25f;
        case EyeState::WAKING:    return 0.6f;
        case EyeState::SCANNING:  return 0.75f;
        case EyeState::IDLE:      return 0.85f;
        case EyeState::TRACKING:  return 1.0f;
        case EyeState::FOCUS:     return 0.9f;  // intense but not wide-eyed
        default:                   return 0.75f;
    }
}

// ── _handleBlink ─────────────────────────────────────────────────────────────
void Behaviour::_handleBlink(uint32_t now) {
    // Blink rate increases when idle/scanning, slows during focus
    uint32_t interval = behaviour.blinkIntervalMs;
    if (_state == EyeState::FOCUS)    interval *= 2;   // focused = less blinking
    if (_state == EyeState::SCANNING) interval = (uint32_t)(interval * 0.8f);

    // Use signed arithmetic to avoid uint32_t underflow on subtraction
    int32_t jitter = (int32_t)random(-(int32_t)behaviour.blinkJitterMs,
                                      (int32_t)behaviour.blinkJitterMs);
    uint32_t blinkInterval = (uint32_t)max(500L, (long)interval + jitter);
    if (now - _lastBlink > blinkInterval) {
        eye.blink();
        _lastBlink = now;
    }
}

// ── State handlers ────────────────────────────────────────────────────────────

void Behaviour::_doSleeping(uint32_t now) {
    eye.setSleeping();
    // Wake on any detection arriving
    DetectionResult det;
    if (xQueuePeek(detectionQueue, &det, 0) == pdTRUE && det.valid) {
        _setState(EyeState::WAKING);
    }
}

void Behaviour::_doWaking(uint32_t now) {
    eye.setGazeDeg(PAN_CENTER, TILT_CENTER, 0.03f);
    eye.setArousal(_arousalForState(EyeState::WAKING), 0.02f);
    uint32_t elapsed = now - _stateMs;
    if (elapsed > behaviour.wakeDelayMs + 800) {
        _setState(EyeState::SCANNING);
    }
}

void Behaviour::_doScanning(uint32_t now) {
    // Two-frequency sinusoidal scan — feels organic, not mechanical
    float dt = 0.020f;  // 50Hz tick
    _scanT += dt;
    float pan  = PAN_CENTER
               + sinf(_scanT * 0.28f) * behaviour.scanPanAmp
               + sinf(_scanT * 0.61f) * (behaviour.scanPanAmp * 0.25f);
    float tilt = TILT_CENTER
               + sinf(_scanT * 0.19f) * behaviour.scanTiltAmp;
    eye.setGazeDeg(pan, tilt, behaviour.gazeAlphaIdle);
    eye.setArousal(_arousalForState(EyeState::SCANNING), 0.04f);

    // Drift toward dozing if nothing detected for a long time
    uint32_t elapsed = now - _stateMs;
    if (elapsed > behaviour.idleToDozeMs) {
        _setState(EyeState::DOZING);
    }
}

void Behaviour::_doIdle(uint32_t now) {
    // Subtle drift near last known position
    float pan  = PAN_CENTER + (_lastNormX - 0.5f) * (PAN_MAX - PAN_MIN) * 0.3f;
    float tilt = TILT_CENTER + (_lastNormY - 0.5f) * (TILT_MAX - TILT_MIN) * 0.3f;
    eye.setGazeDeg(pan, tilt, 0.02f);
    eye.setArousal(_arousalForState(EyeState::IDLE), 0.04f);

    uint32_t elapsed = now - _stateMs;
    if (elapsed > 5000) _setState(EyeState::SCANNING);
}

void Behaviour::_doDozing(uint32_t now) {
    eye.setDozing();
    float dt = 0.020f;
    _scanT += dt * 0.3f;  // very slow scan
    float pan = PAN_CENTER + sinf(_scanT * 0.1f) * behaviour.scanPanAmp * 0.4f;
    eye.setGazeDeg(pan, TILT_CENTER, 0.02f);

    uint32_t elapsed = now - _stateMs;
    if (elapsed > behaviour.dozeToSleepMs) _setState(EyeState::SLEEPING);
}

void Behaviour::_doTracking(uint32_t now, const DetectionResult& det) {
    // Map camera normalised coords → servo degrees
    eye.setGazeNorm(det.normX, det.normY, behaviour.gazeAlphaTrack);
    eye.setArousal(_arousalForState(EyeState::TRACKING), 0.08f);

    // Stamp habituation so we get bored of this location
    saliency.habituate(det.normX, det.normY);

    // Sustained detection → FOCUS
    uint32_t elapsed = now - _stateMs;
    if (elapsed > 3000 && det.confidence > 0.5f) {
        _setState(EyeState::FOCUS);
    }
}

void Behaviour::_doFocus(uint32_t now, const DetectionResult& det) {
    // Locked gaze with very slow alpha — intense stillness
    eye.setGazeNorm(det.normX, det.normY, behaviour.gazeAlphaFocus);
    eye.setArousal(_arousalForState(EyeState::FOCUS), 0.03f);

    // Heavy habituation — eye will eventually tire of this target
    saliency.habituate(det.normX, det.normY);
    saliency.habituate(det.normX, det.normY);  // double stamp in focus
}

// ── update ────────────────────────────────────────────────────────────────────
void Behaviour::update() {
    uint32_t now = millis();

    // ── Read latest detection (non-blocking) ──────────────────────────────────
    DetectionResult det = {false, 0.5f, 0.5f, 0.0f, 0};
    bool gotDet = (detectionQueue != nullptr) &&
                  (xQueuePeek(detectionQueue, &det, 0) == pdTRUE);

    // Track last seen position and time
    if (gotDet && det.valid) {
        _lastNormX  = det.normX;
        _lastNormY  = det.normY;
        _lastConf   = det.confidence;
        _lastDetect = now;
    }

    bool targetPresent = gotDet && det.valid;
    bool targetRecent  = (now - _lastDetect) < 2000;  // linger 2s after loss

    // ── State transitions ─────────────────────────────────────────────────────
    switch (_state) {

        case EyeState::SLEEPING:
            _doSleeping(now);
            break;

        case EyeState::WAKING:
            _doWaking(now);
            if (targetPresent) _setState(EyeState::TRACKING);
            break;

        case EyeState::SCANNING:
            _doScanning(now);
            if (targetPresent) _setState(EyeState::TRACKING);
            break;

        case EyeState::IDLE:
            _doIdle(now);
            if (targetPresent) _setState(EyeState::TRACKING);
            break;

        case EyeState::TRACKING:
            if (targetPresent) {
                _doTracking(now, det);
            } else if (targetRecent) {
                // Linger briefly at last position
                DetectionResult linger = {true, _lastNormX, _lastNormY,
                                          _lastConf * 0.5f, _lastDetect};
                _doTracking(now, linger);
            } else {
                _setState(EyeState::IDLE);
            }
            break;

        case EyeState::FOCUS:
            if (targetPresent) {
                _doFocus(now, det);
            } else if (targetRecent) {
                _setState(EyeState::TRACKING);
            } else {
                _setState(EyeState::IDLE);
            }
            break;

        case EyeState::DOZING:
            _doDozing(now);
            if (targetPresent) _setState(EyeState::WAKING);
            break;
    }

    // ── Blink (all states except sleeping) ───────────────────────────────────
    if (_state != EyeState::SLEEPING) _handleBlink(now);
}