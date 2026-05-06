#include "servo_eye.h"

ServoEye eye;

// ── begin ─────────────────────────────────────────────────────────────────────
bool ServoEye::begin() {
    if (!servoBoard.begin()) return false;

    _curPan = _tgtPan = PAN_CENTER;
    _curTilt = _tgtTilt = TILT_CENTER;
    _curArousal = _tgtArousal = 1.0f;   // start fully open
    _lastPan = _lastTilt = _lastTop = _lastBot = -999.f;

    // Write home position immediately
    servoBoard.setServoAngle(SERVO_CH_PAN,  PAN_CENTER);
    servoBoard.setServoAngle(SERVO_CH_TILT, TILT_CENTER);
    uint8_t topDeg, botDeg;
    _computeLidAngles(TILT_CENTER, PAN_CENTER,
                      _arousalToGap(1.0f), topDeg, botDeg);
    servoBoard.setServoAngle(SERVO_CH_LID_TOP, topDeg);
    servoBoard.setServoAngle(SERVO_CH_LID_BOT, botDeg);

    Serial.println("[ServoEye] Ready — lids open, gaze neutral");
    return true;
}

// ── setGazeNorm ───────────────────────────────────────────────────────────────
void ServoEye::setGazeNorm(float normX, float normY, float alpha) {
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

// ── setArousal ────────────────────────────────────────────────────────────────
void ServoEye::setArousal(float level, float alpha) {
    _tgtArousal = constrain(level, 0.0f, 1.0f);
    _alphaLid   = constrain(alpha, 0.001f, 1.0f);
}

// ── blink ─────────────────────────────────────────────────────────────────────
void ServoEye::blink() {
    if (_blinking) return;
    _blinking            = true;
    _blinkStart          = millis();
    _arousalBeforeBlink  = _tgtArousal;
}

// ── Convenience presets ───────────────────────────────────────────────────────
void ServoEye::setSleeping() {
    setArousal(0.0f, 0.03f);
    setGazeDeg(PAN_CENTER, TILT_CENTER, 0.02f);
}

void ServoEye::setDozing() {
    setArousal(0.3f, 0.03f);
}

void ServoEye::setAwake() {
    setArousal(1.0f, 0.06f);
}

// ── update ────────────────────────────────────────────────────────────────────
bool ServoEye::update() {
    // ── Gaze EMA ─────────────────────────────────────────────────────────────
    _curPan  += _alphaGaze * (_tgtPan  - _curPan);
    _curTilt += _alphaGaze * (_tgtTilt - _curTilt);

    // ── Arousal / blink ───────────────────────────────────────────────────────
    if (_blinking) {
        uint32_t elapsed = millis() - _blinkStart;
        uint32_t half    = behaviour.blinkDurationMs / 2;
        if (elapsed < half) {
            // closing: arousal drops to 0
            _curArousal = _arousalBeforeBlink
                        * (1.0f - (float)elapsed / half);
        } else if (elapsed < behaviour.blinkDurationMs) {
            // opening: arousal returns to pre-blink level
            _curArousal = _arousalBeforeBlink
                        * ((float)(elapsed - half) / half);
        } else {
            _curArousal = _arousalBeforeBlink;
            _blinking   = false;
        }
    } else {
        _curArousal += _alphaLid * (_tgtArousal - _curArousal);
    }

    // ── Pan widening: lids open slightly more when looking sideways ───────────
    float panNorm  = fabsf(_curPan - PAN_CENTER) / (float)(PAN_MAX - PAN_CENTER);
    float widenGap = panNorm * (float)(PAN_MAX - PAN_CENTER) * LID_PAN_WIDEN * 0.1f;
    float gap      = _arousalToGap(_curArousal) + widenGap;

    // ── Compute and write lid angles ──────────────────────────────────────────
    uint8_t topDeg, botDeg;
    _computeLidAngles(_curTilt, _curPan, gap, topDeg, botDeg);
    _writeIfChanged(SERVO_CH_LID_TOP, (float)topDeg, _lastTop);
    _writeIfChanged(SERVO_CH_LID_BOT, (float)botDeg, _lastBot);
    _writeIfChanged(SERVO_CH_PAN,  _curPan,  _lastPan);
    _writeIfChanged(SERVO_CH_TILT, _curTilt, _lastTilt);

    return true;
}

// ── Private helpers ───────────────────────────────────────────────────────────

float ServoEye::_normToPan(float n) const {
    n = constrain(n, 0.0f, 1.0f);
    return PAN_MAX - n * (PAN_MAX - PAN_MIN);
}

float ServoEye::_normToTilt(float n) const {
    n = constrain(n, 0.0f, 1.0f);
    return TILT_MAX - n * (TILT_MAX - TILT_MIN);
}

// Map arousal 0–1 to lid gap in degrees
float ServoEye::_arousalToGap(float arousal) const {
    arousal = constrain(arousal, 0.0f, 1.0f);
    return LID_HALF_GAP_CLOSED
         + arousal * (LID_HALF_GAP_OPEN - LID_HALF_GAP_CLOSED);
}

void ServoEye::_computeLidAngles(float tiltDeg, float panDeg, float gap,
                                  uint8_t& topDeg, uint8_t& botDeg) const {
    // Center tracks tilt — eye looking up shifts both lids up
    float tiltOffset = (tiltDeg - TILT_CENTER) * LID_TILT_FOLLOW;
    float center     = LID_CENTER_DEG - tiltOffset;

    float topF = center + LID_TOP_DIR * gap;
    float botF = center + LID_BOT_DIR * gap;

    topDeg = (uint8_t)constrain(topF, 0.0f, 180.0f);
    botDeg = (uint8_t)constrain(botF, 0.0f, 180.0f);
}

void ServoEye::_writeIfChanged(uint8_t ch, float deg,
                                float& last, float threshold) {
    if (fabsf(deg - last) >= threshold) {
        servoBoard.setServoAngle(ch, (uint8_t)constrain(deg, 0.0f, 180.0f));
        last = deg;
    }
}