#include "servo_eye.h"

ServoEye eye;

// ── begin ─────────────────────────────────────────────────────────────────────
bool ServoEye::begin() {
    // Wire.begin() must be called before this
    if (!servoBoard.begin()) return false;

    // Home all axes to safe starting positions before motion begins
    servoBoard.setServoAngle(SERVO_CH_PAN,     PAN_CENTER);
    servoBoard.setServoAngle(SERVO_CH_TILT,    TILT_CENTER);

    uint8_t topClosed, botClosed;
    _lidToDeg(0.0f, topClosed, botClosed);
    servoBoard.setServoAngle(SERVO_CH_LID_TOP, topClosed);
    servoBoard.setServoAngle(SERVO_CH_LID_BOT, botClosed);

    _curPan  = _tgtPan  = PAN_CENTER;
    _curTilt = _tgtTilt = TILT_CENTER;
    _curLid  = _tgtLid  = 0.0f;
    _lastPan = _lastTilt = _lastLid = -999.f;

    Serial.println("[ServoEye] Homed to neutral/closed");
    return true;
}

// ── setGazeTarget ─────────────────────────────────────────────────────────────
void ServoEye::setGazeTarget(float normX, float normY, float alpha) {
    _tgtPan    = _normToPan(normX);
    _tgtTilt   = _normToTilt(normY);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// ── setGazeDeg ────────────────────────────────────────────────────────────────
void ServoEye::setGazeDeg(float panDeg, float tiltDeg, float alpha) {
    _tgtPan    = constrain(panDeg,  (float)PAN_MIN,  (float)PAN_MAX);
    _tgtTilt   = constrain(tiltDeg, (float)TILT_MIN, (float)TILT_MAX);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// ── setLids ───────────────────────────────────────────────────────────────────
void ServoEye::setLids(float fraction, float alpha) {
    _tgtLid   = constrain(fraction, 0.0f, 1.0f);
    _alphaLid = constrain(alpha, 0.001f, 1.0f);
}

// ── blink ─────────────────────────────────────────────────────────────────────
void ServoEye::blink() {
    if (_blinking) return;
    _blinking       = true;
    _blinkStart     = millis();
    _lidBeforeBlink = _tgtLid;
}

// ── setSleeping / setDozing / setAwake ────────────────────────────────────────
void ServoEye::setSleeping() {
    setLids(0.0f, 0.05f);
    setGazeDeg(PAN_CENTER, TILT_CENTER, 0.02f);
}

void ServoEye::setDozing() {
    setLids(0.35f, 0.04f);
}

void ServoEye::setAwake() {
    setLids(1.0f, 0.06f);
}

// ── update ────────────────────────────────────────────────────────────────────
bool ServoEye::update() {
    // ── Blink state machine ──────────────────────────────────────────────────
    if (_blinking) {
        uint32_t elapsed = millis() - _blinkStart;
        uint32_t half    = behaviour.blinkDurationMs / 2;
        float blinkLid;
        if (elapsed < half) {
            blinkLid = _lidBeforeBlink * (1.0f - (float)elapsed / half);
        } else if (elapsed < behaviour.blinkDurationMs) {
            blinkLid = _lidBeforeBlink * ((float)(elapsed - half) / half);
        } else {
            blinkLid  = _lidBeforeBlink;
            _blinking = false;
        }
        _curLid = blinkLid;
    } else {
        _curLid += _alphaLid * (_tgtLid - _curLid);
    }

    // ── Gaze EMA ────────────────────────────────────────────────────────────
    _curPan  += _alphaGaze * (_tgtPan  - _curPan);
    _curTilt += _alphaGaze * (_tgtTilt - _curTilt);

    // ── Write to hardware only when value has moved enough ───────────────────
    _writeIfChanged(SERVO_CH_PAN,  _curPan,  _lastPan);
    _writeIfChanged(SERVO_CH_TILT, _curTilt, _lastTilt);

    uint8_t topDeg, botDeg;
    _lidToDeg(_curLid, topDeg, botDeg);
    float topF = (float)topDeg;
    if (fabsf(topF - _lastLid) >= 0.6f) {
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, topDeg);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, botDeg);
        _lastLid = topF;
    }

    return true;
}

// ── Private helpers ───────────────────────────────────────────────────────────
float ServoEye::_normToPan(float n) const {
    n = constrain(n, 0.0f, 1.0f);
    return PAN_MAX - n * (PAN_MAX - PAN_MIN);
}

float ServoEye::_normToTilt(float n) const {
    n = constrain(n, 0.0f, 1.0f);
    return TILT_MIN + n * (TILT_MAX - TILT_MIN);
}

void ServoEye::_lidToDeg(float frac, uint8_t& topDeg, uint8_t& botDeg) const {
    frac = constrain(frac, 0.0f, 1.0f);
    topDeg = (uint8_t)(LID_TOP_CLOSED + frac * (LID_TOP_OPEN - LID_TOP_CLOSED));
    botDeg = (uint8_t)(LID_BOT_CLOSED + frac * (LID_BOT_OPEN - LID_BOT_CLOSED));
}

void ServoEye::_writeIfChanged(uint8_t ch, float deg,
                                float& lastWritten, float threshold) {
    if (fabsf(deg - lastWritten) >= threshold) {
        servoBoard.setServoAngle(ch, (uint8_t)constrain(deg, 0.0f, 180.0f));
        lastWritten = deg;
    }
}