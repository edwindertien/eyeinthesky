#pragma once
#include <Arduino.h>
#include "config.h"
#include "m5servo8.h"

// ===========================================================================
//  servo_eye.h -- animated eye abstraction
//
//  Three parameters control the eye:
//
//    X (pan)     -- horizontal gaze
//                  setGazeNorm(x, y)  : 0.0=left  0.5=center  1.0=right
//                  setGazeDeg(pan, tilt) : degrees within mechanical limits
//
//    Y (tilt)    -- vertical gaze
//                  setGazeNorm(x, y)  : 0.0=up    0.5=center  1.0=down
//
//    Arousal     -- eyelid openness
//                  setArousal(0.0) = fully closed / sleeping
//                  setArousal(0.5) = drowsy / half-open
//                  setArousal(1.0) = fully open / wide-eyed / alert
//
//  Lids are automatic -- they follow gaze (tilt tracking, pan widening)
//  without any extra calls. Just set gaze + arousal and call update().
//
//  blink() triggers a brief arousal dip to 0 and back -- non-blocking.
// ===========================================================================

class ServoEye {
public:
    bool begin();

    // -- Gaze ----------------------------------------------------------------
    // Normalised: x/y 0.0-1.0 (e.g. from camera frame)
    void setGazeNorm(float normX, float normY, float alpha = 0.1f);
    // Degrees: pan/tilt within mechanical limits defined in config.h
    void setGazeDeg(float panDeg, float tiltDeg, float alpha = 0.1f);

    // -- Arousal --------------------------------------------------------------
    // 0.0 = closed/sleeping  0.5 = drowsy  1.0 = fully open/alert
    void setArousal(float level, float alpha = 0.06f);

    // -- Blink ----------------------------------------------------------------
    // Non-blocking brief close->open. Ignored if already blinking.
    void blink();

    // -- Convenience presets --------------------------------------------------
    void setSleeping();   // arousal -> 0.0 slowly, gaze drifts to neutral
    void setDozing();     // arousal -> 0.3 slowly
    void setAwake();      // arousal -> 1.0
    void cancelBlink();   // abort any in-progress blink immediately

    // -- Must be called every loop tick ---------------------------------------
    bool update();

    // -- Accessors (for serial reporting / web UI) ----------------------------
    float getPanDeg()   const { return _curPan;     }
    float getTiltDeg()  const { return _curTilt;    }
    float getArousal()  const { return _curArousal; }
    bool  isBlinking()  const { return _blinking;   }

private:
    // Gaze
    float _curPan    = PAN_CENTER;
    float _curTilt   = TILT_CENTER;
    float _tgtPan    = PAN_CENTER;
    float _tgtTilt   = TILT_CENTER;
    float _alphaGaze = 0.1f;
    float _lastPan   = -999.f;
    float _lastTilt  = -999.f;

    // Arousal / lid gap
    float _curArousal = 1.0f;    // start fully open
    float _tgtArousal = 1.0f;
    float _alphaLid   = 0.06f;
    float _lastTop    = -999.f;
    float _lastBot    = -999.f;

    // Blink
    bool     _blinking          = false;
    uint32_t _blinkStart        = 0;
    float    _arousalBeforeBlink = 1.0f;

    // Helpers
    float  _normToPan(float n)  const;
    float  _normToTilt(float n) const;
    void   _computeLidAngles(float tiltDeg, float panDeg, float arousal,
                              uint8_t& topDeg, uint8_t& botDeg) const;
    void   _writeIfChanged(uint8_t ch, float deg, float& last,
                           float threshold = 0.6f);
};

extern ServoEye eye;