#pragma once
#include <Arduino.h>
#include "states.h"
#include "config.h"
#include "servo_eye.h"
#include "blob_tracker.h"

// ═══════════════════════════════════════════════════════════════════════════
//  behaviour.h — eye state machine driven by blob tracker
//
//  States:
//    SLEEPING  — lids closed, no motion detected for a long time
//    WAKING    — motion detected, lids opening
//    SCANNING  — lazy scan, no blobs but recently awake
//    TRACKING  — locked onto a blob, dwelling then shifting
//    FOCUS     — sustained gaze on one blob (long dwell)
//    DOZING    — few/no blobs, drifting toward sleep
//
//  Blob selection:
//    Round-robin between active blobs.
//    Each blob gets a dwell of DWELL_MIN..DWELL_MAX ms.
//    After dwell, shift to next blob (prefer large travel).
//    Blob disappears when person stops moving → background absorbs it.
// ═══════════════════════════════════════════════════════════════════════════

class Behaviour {
public:
    void begin();
    void update();
    EyeState getState() const { return _state; }

private:
    EyeState _state   = EyeState::SCANNING;
    uint32_t _stateMs = 0;

    // Committed gaze
    float    _gazeX       = 0.5f;  // where eye is currently pointed (normalised)
    float    _gazeY       = 0.5f;
    uint32_t _dwellStartMs = 0;    // when we committed to current blob
    uint8_t  _targetBlobId = 0;    // which blob ID we're tracking (0=none)

    // Blob round-robin
    int      _lastBlobIdx  = 0;    // index into last result's blob array

    // Startle
    uint32_t _startleMs    = 0;

    // Blink
    uint32_t _lastBlinkMs  = 0;

    // Last detection
    uint32_t _lastMotionMs = 0;
    uint32_t _wakeResetMs  = 0;   // when resetBackground() was last called on wake
    bool     _wakeResetDone = false; // second reset (post-open) has fired

    // Scan oscillator
    float    _scanT = 0.0f;

    void _setState(EyeState s);
    float _arousalForState(EyeState s) const;
    void _handleBlink(uint32_t now);
    void _doSleeping(uint32_t now, const BlobResult& blobs);
    void _doWaking(uint32_t now);
    void _doScanning(uint32_t now, const BlobResult& blobs);
    void _doTracking(uint32_t now, const BlobResult& blobs);
    void _doDozing(uint32_t now, const BlobResult& blobs);

    // Pick next blob to look at (round-robin, prefer large travel)
    int _pickNextBlob(const BlobResult& blobs) const;

    // Has dwell time expired for current target?
    bool _dwellExpired(uint32_t now) const;
};

extern Behaviour behaviour_sm;