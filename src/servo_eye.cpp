#include "servo_eye.h"

// Singleton definition
ServoEye eye;

// ── begin ─────────────────────────────────────────────────────────────────────
bool ServoEye::begin() {
    // Power up the servo rail
    servoBoard.powerOn();

    // Write safe starting positions before enabling
    // All at neutral / closed so mechanism doesn't snap on power-up
    servoBoard.setAngle(SERVO_CH_PAN,     PAN_CENTER);
    servoBoard.setAngle(SERVO_CH_TILT,    TILT_CENTER);

    uint8_t topClosed, botClosed;
    _lidToDeg(0.0f, topClosed, botClosed);
    servoBoard.setAngle(SERVO_CH_LID_TOP, topClosed);
    servoBoard.setAngle(SERVO_CH_LID_BOT, botClosed);

    // Seed current state so first update() produces no jump
    _curPan  = _tgtPan  = PAN_CENTER;
    _curTilt = _tgtTilt = TILT_CENTER;
    _curLid  = _tgtLid  = 0.0f;
    _lastPan = _lastTilt = _lastLid = -999.f; // force write on first update

    Serial.println("[ServoEye] Initialised at neutral/closed");
    return true;
}

// ── setGazeTarget ─────────────────────────────────────────────────────────────
void ServoEye::setGazeTarget(float normX, float normY, float alpha) {
    _tgtPan   = _normToPan(normX);
    _tgtTilt  = _normToTilt(normY);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// ── setGazeDeg ────────────────────────────────────────────────────────────────
void ServoEye::setGazeDeg(float panDeg, float tiltDeg, float alpha) {
    _tgtPan   = constrain(panDeg,  PAN_MIN,  PAN_MAX);
    _tgtTilt  = constrain(tiltDeg, TILT_MIN, TILT_MAX);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// ── setLids ───────────────────────────────────────────────────────────────────
void ServoEye::setLids(float fraction, float alpha) {
    _tgtLid   = constrain(fraction, 0.0f, 1.0f);
    _alphaLid = constrain(alpha, 0.001f, 1.0f);
}

// ── blink ─────────────────────────────────────────────────────────────────────
void ServoEye::blink() {
    if (_blinking) return;              // ignore if already blinking
    _blinking        = true;
    _blinkStart      = millis();
    _lidBeforeBlink  = _tgtLid;        // restore to this after blink
}

// ── setSleeping / setDozing / setAwake ────────────────────────────────────────
void ServoEye::setSleeping() {
    setLids(0.0f, 0.05f);              // slow close
    setGazeDeg(PAN_CENTER, TILT_CENTER, 0.02f); // drift to neutral
}

void ServoEye::setDozing() {
    setLids(0.35f, 0.04f);             // half-closed, slow
}

void ServoEye::setAwake() {
    setLids(1.0f, 0.06f);
}

// ── update ────────────────────────────────────────────────────────────────────
bool ServoEye::update() {
    bool changed = false;

    // ── Blink state machine ──────────────────────────────────────────────────
    if (_blinking) {
        uint32_t elapsed = millis() - _blinkStart;
        uint32_t half    = behaviour.blinkDurationMs / 2;

        float blinkLid;
        if (elapsed < half) {
            // Closing phase: lid fraction goes from current toward 0
            blinkLid = _lidBeforeBlink * (1.0f - (float)elapsed / half);
        } else if (elapsed < behaviour.blinkDurationMs) {
            // Opening phase: lid fraction returns to pre-blink target
            blinkLid = _lidBeforeBlink * ((float)(elapsed - half) / half);
        } else {
            // Done
            blinkLid  = _lidBeforeBlink;
            _blinking = false;
        }
        // Override lid target during blink
        _curLid = blinkLid;

    } else {
        // Normal lid EMA
        _curLid += _alphaLid * (_tgtLid - _curLid);
    }

    // ── Gaze EMA ────────────────────────────────────────────────────────────
    _curPan  += _alphaGaze * (_tgtPan  - _curPan);
    _curTilt += _alphaGaze * (_tgtTilt - _curTilt);

    // ── Write to hardware (only when values have moved enough) ───────────────
    _writeIfChanged(SERVO_CH_PAN,     _curPan,  _lastPan);
    _writeIfChanged(SERVO_CH_TILT,    _curTilt, _lastTilt);

    // Lid: convert fraction to top/bottom servo degrees then write
    uint8_t topDeg, botDeg;
    _lidToDeg(_curLid, topDeg, botDeg);

    float topF = (float)topDeg;
    float botF = (float)botDeg;
    _writeIfChanged(SERVO_CH_LID_TOP, topF, _lastLid);          // reuse _lastLid
    // Note: we only track one lid for change detection since they always
    // move together. If independent control is needed, add _lastLidBot.
    servoBoard.setAngle(SERVO_CH_LID_BOT, botDeg);

    return changed;
}

// ── Private helpers ───────────────────────────────────────────────────────────

float ServoEye::_normToPan(float n) const {
    // n=0 → left (PAN_MAX), n=1 → right (PAN_MIN)
    // Camera is wide-angle; flip X if your mount is mirrored
    n = constrain(n, 0.0f, 1.0f);
    return PAN_MAX - n * (PAN_MAX - PAN_MIN);
}

float ServoEye::_normToTilt(float n) const {
    // n=0 → top of frame → tilt up (TILT_MIN), n=1 → bottom → tilt down
    n = constrain(n, 0.0f, 1.0f);
    return TILT_MIN + n * (TILT_MAX - TILT_MIN);
}

void ServoEye::_lidToDeg(float frac, uint8_t& topDeg, uint8_t& botDeg) const {
    frac = constrain(frac, 0.0f, 1.0f);
    // Interpolate between closed and open positions defined in config.h
    topDeg = (uint8_t)(LID_TOP_CLOSED + frac * (LID_TOP_OPEN - LID_TOP_CLOSED));
    botDeg = (uint8_t)(LID_BOT_CLOSED + frac * (LID_BOT_OPEN - LID_BOT_CLOSED));
}

void ServoEye::_writeIfChanged(uint8_t ch, float deg,
                                float& lastWritten, float threshold) {
    if (fabsf(deg - lastWritten) >= threshold) {
        servoBoard.setAngle(ch, (uint8_t)constrain(deg, 0.0f, 180.0f));
        lastWritten = deg;
    }
}
