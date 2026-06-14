#include "calibration.h"
#include "servo_eye.h"

ServoEye eye;

// -- begin ---------------------------------------------------------------------
bool ServoEye::begin() {
    if (!servoBoard.begin()) return false;

    // Safety clamp: regardless of calibration source, never write outside
    // the absolute safe range for an SG90 (20-160 deg).
    // This is the last line of defence against corrupted calibration data.
    auto clampSafe = [](uint8_t v) -> uint8_t {
        return (uint8_t)constrain((int)v, 20, 160);
    };
    uint8_t safeCenter = clampSafe(calibration.data.panCenter);
    uint8_t safeTiltC  = clampSafe(calibration.data.tiltCenter);

    _curPan = _tgtPan = safeCenter;
    _curTilt = _tgtTilt = calibration.data.tiltCenter;
    _curArousal = _tgtArousal = 1.0f;   // start fully open
    _lastPan = _lastTilt = _lastTop = _lastBot = -999.f;

    // Write home position immediately
    servoBoard.setServoAngle(SERVO_CH_PAN,  safeCenter);
    servoBoard.setServoAngle(SERVO_CH_TILT, safeTiltC);
    uint8_t topDeg, botDeg;
    _computeLidAngles(calibration.data.tiltCenter, calibration.data.panCenter,
                      0.5f, topDeg, botDeg);  // 0.5 = rest position
    servoBoard.setServoAngle(SERVO_CH_LID_TOP, topDeg);
    servoBoard.setServoAngle(SERVO_CH_LID_BOT, botDeg);

    Serial.println("[ServoEye] Ready -- lids open, gaze neutral");
    return true;
}

// -- setGazeNorm ---------------------------------------------------------------
void ServoEye::setGazeNorm(float normX, float normY, float alpha) {
    _tgtPan    = _normToPan(normX);
    _tgtTilt   = _normToTilt(normY);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// -- setGazeDeg ----------------------------------------------------------------
void ServoEye::setGazeDeg(float panDeg, float tiltDeg, float alpha) {
    _tgtPan    = constrain(panDeg,  (float)calibration.data.panMin,  (float)calibration.data.panMax);
    _tgtTilt   = constrain(tiltDeg, (float)calibration.data.tiltMin, (float)calibration.data.tiltMax);
    _alphaGaze = constrain(alpha, 0.001f, 1.0f);
}

// -- setArousal ----------------------------------------------------------------
void ServoEye::setArousal(float level, float alpha) {
    _tgtArousal = constrain(level, 0.0f, 1.0f);
    _alphaLid   = constrain(alpha, 0.001f, 1.0f);
}

// -- blink ---------------------------------------------------------------------
void ServoEye::blink() {
    if (_blinking) return;
    _blinking            = true;
    _blinkStart          = millis();
    _arousalBeforeBlink  = _tgtArousal;
}

// -- Convenience presets -------------------------------------------------------
void ServoEye::setSleeping() {
    setArousal(0.0f, 0.03f);
    setGazeDeg(calibration.data.panCenter, calibration.data.tiltCenter, 0.02f);
}

void ServoEye::setDozing() {
    setArousal(0.3f, 0.03f);
}

void ServoEye::setAwake() {
    setArousal(1.0f, 0.06f);
}

// -- update --------------------------------------------------------------------
bool ServoEye::update() {
    // -- Gaze -- distance-weighted alpha ---------------------------------------
    // Large movements use a higher alpha (snappier), small corrections use lower.
    // This gives decisive gaze shifts to corners while still being smooth near target.
    float panErr  = _tgtPan  - _curPan;
    float tiltErr = _tgtTilt - _curTilt;
    float dist    = sqrtf(panErr*panErr + tiltErr*tiltErr);

    // Scale alpha: at 30 deg distance use full alpha, at <2 deg use minimum
    // This means: pick a corner -> snap there -> settle gently
    float dynamicAlpha = _alphaGaze;
    if (dist > 5.0f) {
        // Boost alpha proportionally to distance, capped at 0.35
        dynamicAlpha = fminf(0.35f, _alphaGaze + (dist / 30.0f) * 0.25f);
    }

    _curPan  += dynamicAlpha * panErr;
    _curTilt += dynamicAlpha * tiltErr;

    // -- Arousal / blink -------------------------------------------------------
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

    // -- Pan widening: lids open slightly more when looking sideways -----------
    float panNorm  = fabsf(_curPan - calibration.data.panCenter) / (float)(calibration.data.panMax - calibration.data.panCenter);
    float widenGap = panNorm * (float)(calibration.data.panMax - calibration.data.panCenter) * calibration.data.lidPanWiden * 0.1f;
    // Pass arousal directly (0=closed, 0.5=rest, 1=maxOpen)
    // panOffset passed separately via widenGap (unused in new model)
    float gap = _curArousal;  // repurposed as arousal in _computeLidAngles

    // -- Compute and write lid angles ------------------------------------------
    uint8_t topDeg, botDeg;
    _computeLidAngles(_curTilt, _curPan, gap, topDeg, botDeg);
    _writeIfChanged(SERVO_CH_LID_TOP, (float)topDeg, _lastTop);
    _writeIfChanged(SERVO_CH_LID_BOT, (float)botDeg, _lastBot);
    _writeIfChanged(SERVO_CH_PAN,  _curPan,  _lastPan);
    _writeIfChanged(SERVO_CH_TILT, _curTilt, _lastTilt);

    return true;
}

// -- Private helpers -----------------------------------------------------------

float ServoEye::_normToPan(float n) const {
    if (camMirrorX) n = 1.0f - n;
    n = 0.5f + (n - 0.5f - camPanOffset) * camPanScale;
    n = constrain(n, 0.0f, 1.0f);
    return calibration.data.panMax
           - n * (calibration.data.panMax - calibration.data.panMin);
}

float ServoEye::_normToTilt(float n) const {
    if (camMirrorY) n = 1.0f - n;
    n = 0.5f + (n - 0.5f - camTiltOffset) * camTiltScale;
    n = constrain(n, 0.0f, 1.0f);
    return calibration.data.tiltMax
           - n * (calibration.data.tiltMax - calibration.data.tiltMin);
}

// Compute lid angles from arousal (0=closed, 0.5=rest, 1=maxOpen)
// and current gaze (tilt shifts lids, pan widens gap slightly)
void ServoEye::_computeLidAngles(float tiltDeg, float panDeg, float gap,
                                  uint8_t& topDeg, uint8_t& botDeg) const {
    // 'gap' parameter repurposed as arousal (0-1) for the new model
    float arousal = constrain(gap, 0.0f, 1.0f);

    float tiltOffset = (tiltDeg - (float)calibration.data.tiltCenter)
                     * calibration.data.lidTiltFollow;
    float panOffset  = fabsf(panDeg - (float)calibration.data.panCenter)
                     / (float)(calibration.data.panMax - calibration.data.panCenter)
                     * calibration.data.lidPanWiden;

    // Interpolate top lid: arousal<0.5 -> closed<->rest, arousal>0.5 -> rest<->maxOpen
    float topF, botF;
    if (arousal <= 0.5f) {
        float t = arousal * 2.0f;   // 0->1 over closed..rest
        topF = calibration.data.lidTopClosed
             + t * ((float)calibration.data.lidTopRest - calibration.data.lidTopClosed);
        botF = calibration.data.lidBotClosed
             + t * ((float)calibration.data.lidBotRest - calibration.data.lidBotClosed);
    } else {
        float t = (arousal - 0.5f) * 2.0f;   // 0->1 over rest..maxOpen
        topF = calibration.data.lidTopRest
             + t * ((float)calibration.data.lidTopMaxOpen - calibration.data.lidTopRest);
        botF = calibration.data.lidBotRest
             + t * ((float)calibration.data.lidBotMaxOpen - calibration.data.lidBotRest);
        // Extra widening on pan at high arousal
        topF -= panOffset * t * 5.0f;
        botF += panOffset * t * 5.0f;
    }

    // Tilt follow: shift both lids proportionally with gaze
    topF -= tiltOffset;
    botF -= tiltOffset;

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