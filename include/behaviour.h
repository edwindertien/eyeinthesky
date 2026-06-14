#pragma once
#include <Arduino.h>
#include "states.h"
#include "config.h"
#include "servo_eye.h"
#include "blob_tracker.h"

// =========================================================================
//  behaviour.h -- eye state machine
//
//  State sequence:
//
//    BOOT -> SLEEPING (lids closed, bg model building)
//         -> (visionBgSettled) WAKING (lids open slowly, bg reset scheduled)
//         -> (bg settled) SCANNING (lazy scan, waiting for stable blobs)
//         -> (blobs + bg settled) TRACKING (committed gaze, dwell/shift)
//         -> (sustained) FOCUS (intense stillness)
//
//    Any state -> SCARED (camera shift detected)
//              -> lids snap shut, wait for settle
//              -> WAKING (cautious reopen)
//
//    SCANNING -> DOZING -> SLEEPING (no motion)
// =========================================================================

class Behaviour {
public:
    void begin();
    void update();
    EyeState getState() const { return _state; }

private:
    EyeState _state    = EyeState::SLEEPING;
    uint32_t _stateMs  = 0;

    // Committed gaze
    float    _gazeX       = 0.5f;
    float    _gazeY       = 0.5f;
    uint32_t _dwellStartMs = 0;
    uint8_t  _targetBlobId = 0;
    bool     _hasCommitted = false;

    // Startle
    uint32_t _startleMs   = 0;

    // Blink
    uint32_t _lastBlinkMs = 0;

    // Last motion
    uint32_t _lastMotionMs = 0;

    // Wake/reset tracking
    uint32_t _wakeResetMs   = 0;
    bool     _wakeResetDone = false;
    bool     _isFirstBoot   = true;   // true until first wake from sleep

    // Scan oscillator
    float    _scanT = 0.0f;

    void _setState(EyeState s);
    float _arousalForState(EyeState s) const;
    void _handleBlink(uint32_t now);
    bool _dwellExpired(uint32_t now) const;
    int  _pickNextBlob(const BlobResult& blobs) const;

    void _doSleeping(uint32_t now, const BlobResult& blobs);
    void _doWaking(uint32_t now);
    void _doScanning(uint32_t now, const BlobResult& blobs);
    void _doTracking(uint32_t now, const BlobResult& blobs);
    void _doFocus(uint32_t now, const BlobResult& blobs);
    void _doDozing(uint32_t now, const BlobResult& blobs);
    void _doScared(uint32_t now, const BlobResult& blobs);
};

extern Behaviour behaviour_sm;