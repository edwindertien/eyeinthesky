#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include "m5servo8.h"

// ═══════════════════════════════════════════════════════════════════════════
//  calibration.h — servo calibration with NVS storage
//
//  Eyeball: pan and tilt each have min / center / max
//
//  Eyelids: 3 positions per lid (all in servo degrees)
//    Closed   — fully shut (blink / sleep)
//    Rest     — normal relaxed open, iris comfortably framed
//    MaxOpen  — widest possible (surprise / high arousal)
//
//  Animation mapping:
//    arousal 0.0 → Closed
//    arousal 0.5 → Rest     (default awake)
//    arousal 1.0 → MaxOpen  (alert / surprised)
//    Tilt offsets all three positions proportionally (lid follows gaze)
//    Pan slightly widens the rest↔maxOpen gap (foreshortening)
//
//  Direction is implicit — whichever angle is numerically smaller is
//  "more closed" for that lid. No direction flag needed.
// ═══════════════════════════════════════════════════════════════════════════

struct CalibData {
    // ── Eyeball ──────────────────────────────────────────────────────────────
    uint8_t panCenter;
    uint8_t panMin;
    uint8_t panMax;
    uint8_t tiltCenter;
    uint8_t tiltMin;
    uint8_t tiltMax;

    // ── Eyelids — 3 positions each ───────────────────────────────────────────
    uint8_t lidTopClosed;    // upper lid: fully shut
    uint8_t lidTopRest;      // upper lid: normal open (iris framed)
    uint8_t lidTopMaxOpen;   // upper lid: wide open (surprise/alert)

    uint8_t lidBotClosed;    // lower lid: fully shut
    uint8_t lidBotRest;      // lower lid: normal open
    uint8_t lidBotMaxOpen;   // lower lid: wide open

    // ── Lid dynamics ─────────────────────────────────────────────────────────
    float   lidTiltFollow;   // how much tilt shifts lids (0=none 1=full 1:1)
    float   lidPanWiden;     // extra gap widening on pan (foreshortening)
};

// Safe defaults — adjust after first flash via calibration mode
static const CalibData CALIB_DEFAULTS = {
    // Pan:  center  min   max
               90,   58,  122,
    // Tilt: center  min   max
               85,   68,  102,
    // Top lid:  closed  rest  maxOpen
                  100,   70,    55,
    // Bot lid:  closed  rest  maxOpen
                   75,  100,   115,
    // Dynamics
    0.6f,   // lidTiltFollow
    0.3f    // lidPanWiden
};

// ── Calibration sub-axes ──────────────────────────────────────────────────────
enum class CalibAxis : uint8_t {
    PAN, TILT,
    LID_TOP,   // upper lid — nudge moves current position
    LID_BOT,   // lower lid
};

enum class LidPos : uint8_t {
    CLOSED, REST, MAX_OPEN
};

class Calibration {
public:
    void begin();
    bool save();
    void reset();
    bool run();     // blocking interactive mode
    void apply();   // push data into servo_eye model

    CalibData data;

private:
    Preferences _prefs;

    CalibAxis _axis    = CalibAxis::PAN;
    LidPos    _lidPos  = LidPos::REST;
    uint8_t   _rawAngle = 90;
    bool      _changed  = false;

    bool _isValid(const CalibData& d) const;

    // Write current servo angle for selected axis
    void _writeServo(uint8_t deg);

    // Write both lids at a specific named position (for preview)
    void _writeLidsAt(LidPos pos);

    // Nudge current axis ±delta degrees
    void _nudge(int delta);

    // Store _rawAngle into whichever field is currently selected
    void _saveCurrentAngle();

    void _printStatus();
    void _printHelp();
    void _testMotion();
    void _testArouse();
    void _handleCommand(const String& cmd);

    // Get/set the angle for current axis+lidPos
    uint8_t  _getCurrentAngle() const;
    uint8_t& _getCurrentAngleRef();
};

extern Calibration calibration;