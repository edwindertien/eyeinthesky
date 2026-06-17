#include "calibration.h"
#include "servo_eye.h"
#include "config.h"

Calibration calibration;

static const char* NVS_NS  = "eyecal";
static const char* NVS_KEY = "caldata";

// -- _isValid ------------------------------------------------------------------
bool Calibration::_isValid(const CalibData& d) const {
    auto inRange = [](uint8_t v, uint8_t lo, uint8_t hi) {
        return v >= lo && v <= hi;
    };
    // Pan: min < center < max, all within 20-160 deg
    if (!inRange(d.panMin,    20, 160)) return false;
    if (!inRange(d.panMax,    20, 160)) return false;
    if (!inRange(d.panCenter, 20, 160)) return false;
    if (d.panMin >= d.panCenter || d.panCenter >= d.panMax) return false;
    if (d.panMax - d.panMin < 10) return false;

    // Tilt
    if (!inRange(d.tiltMin,    30, 150)) return false;
    if (!inRange(d.tiltMax,    30, 150)) return false;
    if (!inRange(d.tiltCenter, 30, 150)) return false;
    if (d.tiltMin >= d.tiltCenter || d.tiltCenter >= d.tiltMax) return false;
    if (d.tiltMax - d.tiltMin < 5) return false;

    // Lid angles: all within 10-170 deg
    auto inLid = [&](uint8_t v) { return inRange(v, 10, 170); };
    if (!inLid(d.lidTopClosed)  || !inLid(d.lidTopRest)    ||
        !inLid(d.lidTopMaxOpen) || !inLid(d.lidBotClosed)  ||
        !inLid(d.lidBotRest)    || !inLid(d.lidBotMaxOpen)) return false;

    // Lid positions must be distinct (at least 2 deg between each step)
    if (abs((int)d.lidTopClosed  - (int)d.lidTopRest)    < 2) return false;
    if (abs((int)d.lidTopRest    - (int)d.lidTopMaxOpen) < 2) return false;
    if (abs((int)d.lidBotClosed  - (int)d.lidBotRest)    < 2) return false;
    if (abs((int)d.lidBotRest    - (int)d.lidBotMaxOpen) < 2) return false;

    // Dynamics
    if (d.lidTiltFollow < 0.0f || d.lidTiltFollow > 1.5f) return false;
    if (d.lidPanWiden   < 0.0f || d.lidPanWiden   > 1.0f) return false;

    return true;
}

// -- begin ---------------------------------------------------------------------
void Calibration::begin() {
    _prefs.begin(NVS_NS, true);
    size_t len = _prefs.getBytesLength(NVS_KEY);
    bool loaded = false;
    if (len == sizeof(CalibData)) {
        loaded = (_prefs.getBytes(NVS_KEY, &data, sizeof(CalibData))
                  == sizeof(CalibData));
    }
    _prefs.end();

    _dirty = false;
    if (loaded && _isValid(data)) {
        Serial.println("[Cal] Loaded from NVS");
    } else {
        if (loaded) Serial.println("[Cal] NVS invalid -- using defaults");
        else        Serial.println("[Cal] No NVS data -- using defaults");
        data = CALIB_DEFAULTS;
    }
    work = data;
    apply();
}

// -- save ---------------------------------------------------------------------
bool Calibration::save() {
    if (!_isValid(work)) {
        Serial.println("[Cal] Working values failed validation -- not saved");
        Serial.println("[Cal] Check: min < center < max for pan/tilt");
        return false;
    }
    data = work;   // commit working copy to live data
    _prefs.begin(NVS_NS, false);
    size_t w = _prefs.putBytes(NVS_KEY, &data, sizeof(CalibData));
    _prefs.end();
    bool ok = (w == sizeof(CalibData));
    if (ok) _dirty = false;
    Serial.printf("[Cal] Save %s\n", ok ? "OK" : "FAILED");
    apply();
    return ok;
}

// -- reset ---------------------------------------------------------------------
void Calibration::reset() {
    data = CALIB_DEFAULTS;
    work = CALIB_DEFAULTS;
    _dirty = false;
    _prefs.begin(NVS_NS, false);
    _prefs.clear();
    _prefs.end();
    apply();
    Serial.println("[Cal] Reset to defaults");
}

// -- apply ---------------------------------------------------------------------
void Calibration::apply() {
    Serial.printf("[Cal] pan %d/%d/%d  tilt %d/%d/%d\n",
                  data.panMin, data.panCenter, data.panMax,
                  data.tiltMin, data.tiltCenter, data.tiltMax);
    Serial.printf("[Cal] topLid closed=%d rest=%d maxOpen=%d\n",
                  data.lidTopClosed, data.lidTopRest, data.lidTopMaxOpen);
    Serial.printf("[Cal] botLid closed=%d rest=%d maxOpen=%d\n",
                  data.lidBotClosed, data.lidBotRest, data.lidBotMaxOpen);
}

// -- _getCurrentAngle / Ref ----------------------------------------------------
uint8_t Calibration::_getCurrentAngle() const {
    switch (_axis) {
        case CalibAxis::PAN:
            switch (_panTiltPos) {
                case PanTiltPos::MIN:    return work.panMin;
                case PanTiltPos::MAX:    return work.panMax;
                default:                 return work.panCenter;
            }
        case CalibAxis::TILT:
            switch (_panTiltPos) {
                case PanTiltPos::MIN:    return work.tiltMin;
                case PanTiltPos::MAX:    return work.tiltMax;
                default:                 return work.tiltCenter;
            }
        case CalibAxis::LID_TOP:
            switch (_lidPos) {
                case LidPos::CLOSED:   return work.lidTopClosed;
                case LidPos::REST:     return work.lidTopRest;
                case LidPos::MAX_OPEN: return work.lidTopMaxOpen;
            }
        case CalibAxis::LID_BOT:
            switch (_lidPos) {
                case LidPos::CLOSED:   return work.lidBotClosed;
                case LidPos::REST:     return work.lidBotRest;
                case LidPos::MAX_OPEN: return work.lidBotMaxOpen;
            }
    }
    return 90;
}

uint8_t& Calibration::_getCurrentAngleRef() {
    switch (_axis) {
        case CalibAxis::PAN:
            switch (_panTiltPos) {
                case PanTiltPos::MIN:    return work.panMin;
                case PanTiltPos::MAX:    return work.panMax;
                default:                 return work.panCenter;
            }
        case CalibAxis::TILT:
            switch (_panTiltPos) {
                case PanTiltPos::MIN:    return work.tiltMin;
                case PanTiltPos::MAX:    return work.tiltMax;
                default:                 return work.tiltCenter;
            }
        case CalibAxis::LID_TOP:
            switch (_lidPos) {
                case LidPos::CLOSED:   return work.lidTopClosed;
                case LidPos::REST:     return work.lidTopRest;
                case LidPos::MAX_OPEN: return work.lidTopMaxOpen;
            }
        case CalibAxis::LID_BOT:
            switch (_lidPos) {
                case LidPos::CLOSED:   return work.lidBotClosed;
                case LidPos::REST:     return work.lidBotRest;
                case LidPos::MAX_OPEN: return work.lidBotMaxOpen;
            }
    }
    return work.panCenter;  // fallback
}

// -- _writeServo ---------------------------------------------------------------
void Calibration::_writeServo(uint8_t deg) {
    deg = (uint8_t)constrain(deg, 0, 180);
    switch (_axis) {
        case CalibAxis::PAN:
            servoBoard.setServoAngle(SERVO_CH_PAN, deg);
            break;
        case CalibAxis::TILT:
            servoBoard.setServoAngle(SERVO_CH_TILT, deg);
            break;
        case CalibAxis::LID_TOP:
            servoBoard.setServoAngle(SERVO_CH_LID_TOP, deg);
            break;
        case CalibAxis::LID_BOT:
            servoBoard.setServoAngle(SERVO_CH_LID_BOT, deg);
            break;
    }
    _rawAngle = deg;
}

// -- _writeLidsAt -- write both lids to a named position -----------------------
void Calibration::_writeLidsAt(LidPos pos) {
    uint8_t top, bot;
    switch (pos) {
        case LidPos::CLOSED:
            top = work.lidTopClosed;   bot = work.lidBotClosed;   break;
        case LidPos::REST:
            top = work.lidTopRest;     bot = work.lidBotRest;     break;
        case LidPos::MAX_OPEN:
            top = work.lidTopMaxOpen;  bot = work.lidBotMaxOpen;  break;
        default:
            top = work.lidTopRest;     bot = work.lidBotRest;     break;
    }
    servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
    servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
}

// -- _nudge --------------------------------------------------------------------
// Moves servo AND immediately saves into the currently selected position field.
// Select which field to edit with: center/min/max (pan/tilt) or rest/closed/maxopen (lid)
void Calibration::_nudge(int delta) {
    int next = (int)_rawAngle + delta;
    next = constrain(next, 0, 180);
    _rawAngle = (uint8_t)next;
    _writeServo(_rawAngle);
    // Save into working copy field -- NOT committed until 'save'
    _getCurrentAngleRef() = _rawAngle;
    _dirty = true;
    // Print which field is being updated
    const char* fieldName = "?";
    switch (_axis) {
        case CalibAxis::PAN:
            fieldName = (_panTiltPos == PanTiltPos::CENTER) ? "pan center" :
                        (_panTiltPos == PanTiltPos::MIN)    ? "pan min"    : "pan max";
            break;
        case CalibAxis::TILT:
            fieldName = (_panTiltPos == PanTiltPos::CENTER) ? "tilt center" :
                        (_panTiltPos == PanTiltPos::MIN)    ? "tilt min"   : "tilt max";
            break;
        case CalibAxis::LID_TOP:
            fieldName = (_lidPos == LidPos::CLOSED)   ? "top closed"   :
                        (_lidPos == LidPos::REST)      ? "top rest"     : "top maxOpen";
            break;
        case CalibAxis::LID_BOT:
            fieldName = (_lidPos == LidPos::CLOSED)   ? "bot closed"   :
                        (_lidPos == LidPos::REST)      ? "bot rest"     : "bot maxOpen";
            break;
    }
    Serial.printf("[Cal] %s = %d deg\n", fieldName, _rawAngle);
}

// -- _printStatus --------------------------------------------------------------
void Calibration::_printStatus() {
    const char* axName =
        _axis == CalibAxis::PAN     ? "PAN" :
        _axis == CalibAxis::TILT    ? "TILT" :
        _axis == CalibAxis::LID_TOP ? "LID TOP" : "LID BOT";
    const char* posName =
        _lidPos == LidPos::CLOSED   ? "CLOSED" :
        _lidPos == LidPos::REST     ? "REST" : "MAX_OPEN";

    Serial.printf("\n[Cal] axis=%-7s  pos=%-8s  angle=%3d deg  %s\n",
                  axName, posName, _rawAngle,
                  _dirty ? "* UNSAVED *" : "saved");
    Serial.printf("  Pan : min=%3d  center=%3d  max=%3d\n",
                  work.panMin, work.panCenter, work.panMax);
    Serial.printf("  Tilt: min=%3d  center=%3d  max=%3d\n",
                  work.tiltMin, work.tiltCenter, work.tiltMax);
    Serial.printf("  Top : closed=%3d  rest=%3d  maxOpen=%3d\n",
                  work.lidTopClosed, work.lidTopRest, work.lidTopMaxOpen);
    Serial.printf("  Bot : closed=%3d  rest=%3d  maxOpen=%3d\n",
                  work.lidBotClosed, work.lidBotRest, work.lidBotMaxOpen);
}

// -- _printHelp ----------------------------------------------------------------
void Calibration::_printHelp() {
    Serial.println("\n=========================================");
    Serial.println("       SERVO CALIBRATION MODE");
    Serial.println("=========================================");
    Serial.println(" WORKFLOW:");
    Serial.println("   1. Select axis:    pan | tilt | top | bot");
    Serial.println("   2. Select field:");
    Serial.println("      pan/tilt:       center | min | max");
    Serial.println("      lid:            rest | closed | maxopen");
    Serial.println("   3. Nudge:  +  -  (1 deg)   ++  --  (5 deg)");
    Serial.println("      -> servo moves AND saves immediately");
    Serial.println(" EXAMPLE (tilt):");
    Serial.println("   tilt -> center -> nudge -> min -> nudge -> max -> nudge -> save");
    Serial.println(" EXAMPLE (lid):");
    Serial.println("   top -> rest -> nudge -> closed -> nudge -> maxopen -> nudge -> save");
    Serial.println(" PREVIEW: show rest | show closed | show maxopen");
    Serial.println(" TESTS:   test | blink | arouse");
    Serial.println(" STORE:   save | load | reset | status | exit");
    Serial.println("=========================================");
        _printStatus();
}

// -- _testMotion ---------------------------------------------------------------
void Calibration::_testMotion() {
    Serial.println("[Cal] Testing...");

    Serial.println("  Pan sweep");
    for (int a = work.panMin; a <= work.panMax; a += 2)
        { servoBoard.setServoAngle(SERVO_CH_PAN, a); delay(18); }
    for (int a = work.panMax; a >= work.panMin; a -= 2)
        { servoBoard.setServoAngle(SERVO_CH_PAN, a); delay(18); }
    servoBoard.setServoAngle(SERVO_CH_PAN, work.panCenter);
    delay(300);

    Serial.println("  Tilt sweep");
    for (int a = work.tiltMin; a <= work.tiltMax; a += 2)
        { servoBoard.setServoAngle(SERVO_CH_TILT, a); delay(18); }
    for (int a = work.tiltMax; a >= work.tiltMin; a -= 2)
        { servoBoard.setServoAngle(SERVO_CH_TILT, a); delay(18); }
    servoBoard.setServoAngle(SERVO_CH_TILT, work.tiltCenter);
    delay(300);

    Serial.println("  Lids: maxOpen -> rest -> closed -> rest");
    _writeLidsAt(LidPos::MAX_OPEN); delay(600);
    _writeLidsAt(LidPos::REST);     delay(600);
    _writeLidsAt(LidPos::CLOSED);   delay(400);
    _writeLidsAt(LidPos::REST);     delay(400);

    Serial.println("[Cal] Done");
}

// -- _testArouse -- sweep arousal 0->1 ------------------------------------------
void Calibration::_testArouse() {
    Serial.println("[Cal] Arousal sweep: closed -> rest -> maxOpen -> rest -> closed");
    // closed -> rest
    for (int i = 0; i <= 20; i++) {
        float t = i / 20.0f;
        uint8_t top = work.lidTopClosed + t * (work.lidTopRest - work.lidTopClosed);
        uint8_t bot = work.lidBotClosed + t * (work.lidBotRest - work.lidBotClosed);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // rest -> maxOpen
    for (int i = 0; i <= 20; i++) {
        float t = i / 20.0f;
        uint8_t top = work.lidTopRest + t * (work.lidTopMaxOpen - work.lidTopRest);
        uint8_t bot = work.lidBotRest + t * (work.lidBotMaxOpen - work.lidBotRest);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // back to rest
    for (int i = 20; i >= 0; i--) {
        float t = i / 20.0f;
        uint8_t top = work.lidTopRest + t * (work.lidTopMaxOpen - work.lidTopRest);
        uint8_t bot = work.lidBotRest + t * (work.lidBotMaxOpen - work.lidBotRest);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // rest -> closed
    for (int i = 20; i >= 0; i--) {
        float t = i / 20.0f;
        uint8_t top = work.lidTopClosed + t * (work.lidTopRest - work.lidTopClosed);
        uint8_t bot = work.lidBotClosed + t * (work.lidBotRest - work.lidBotClosed);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    _writeLidsAt(LidPos::REST);
    Serial.println("[Cal] Arouse done");
}

// -- _handleCommand ------------------------------------------------------------
void Calibration::_handleCommand(const String& cmd) {

    // -- Axis selection --------------------------------------------------------
    if (cmd == "pan") {
        _axis       = CalibAxis::PAN;
        _panTiltPos = PanTiltPos::CENTER;   // default to editing center
        _rawAngle   = work.panCenter;
        servoBoard.setServoAngle(SERVO_CH_PAN, _rawAngle);
        Serial.println("[Cal] Axis: PAN -- editing CENTER");
        Serial.println("       type center/min/max to switch field, +/- to nudge");
        _printStatus();
    }
    else if (cmd == "tilt") {
        _axis       = CalibAxis::TILT;
        _panTiltPos = PanTiltPos::CENTER;
        _rawAngle   = work.tiltCenter;
        servoBoard.setServoAngle(SERVO_CH_TILT, _rawAngle);
        Serial.println("[Cal] Axis: TILT -- editing CENTER");
        Serial.println("       type center/min/max to switch field, +/- to nudge");
        _printStatus();
    }
    else if (cmd == "top") {
        _axis     = CalibAxis::LID_TOP;
        _lidPos   = LidPos::REST;
        _rawAngle = work.lidTopRest;
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, _rawAngle);
        Serial.println("[Cal] Axis: LID TOP -- editing REST");
        Serial.println("       type closed/rest/maxopen to switch field, +/- to nudge");
        _printStatus();
    }
    else if (cmd == "bot") {
        _axis     = CalibAxis::LID_BOT;
        _lidPos   = LidPos::REST;
        _rawAngle = work.lidBotRest;
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, _rawAngle);
        Serial.println("[Cal] Axis: LID BOT -- editing REST");
        Serial.println("       type closed/rest/maxopen to switch field, +/- to nudge");
        _printStatus();
    }

    // -- Nudge -----------------------------------------------------------------
    else if (cmd == "+")   _nudge(+1);
    else if (cmd == "-")   _nudge(-1);
    else if (cmd == "++")  _nudge(+5);
    else if (cmd == "--")  _nudge(-5);

    // -- Pan/tilt position select ----------------------------------------------
    // Typing 'center'/'min'/'max' selects which field +/- will edit,
    // and jumps the servo to the currently stored value for that field.
    else if (cmd == "center") {
        _panTiltPos = PanTiltPos::CENTER;
        _rawAngle   = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing CENTER = %d deg  (use +/- to adjust)\n", _rawAngle);
    }
    else if (cmd == "min") {
        _panTiltPos = PanTiltPos::MIN;
        _rawAngle   = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing MIN = %d deg  (use +/- to adjust)\n", _rawAngle);
    }
    else if (cmd == "max") {
        _panTiltPos = PanTiltPos::MAX;
        _rawAngle   = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing MAX = %d deg  (use +/- to adjust)\n", _rawAngle);
    }

    // -- Lid position select + save --------------------------------------------
    // Typing 'closed' / 'rest' / 'maxopen' does two things:
    //   1. Jumps the servo to the currently stored angle for that position
    //   2. Sets _lidPos so subsequent +/- nudges and 'save' apply there
    else if (cmd == "closed") {
        _lidPos   = LidPos::CLOSED;
        _rawAngle = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing CLOSED = %d deg  (use +/- to adjust)\n", _rawAngle);
    }
    else if (cmd == "rest") {
        _lidPos   = LidPos::REST;
        _rawAngle = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing REST = %d deg  (use +/- to adjust)\n", _rawAngle);
    }
    else if (cmd == "maxopen") {
        _lidPos   = LidPos::MAX_OPEN;
        _rawAngle = _getCurrentAngle();
        _writeServo(_rawAngle);
        Serial.printf("[Cal] Editing MAX OPEN = %d deg  (use +/- to adjust)\n", _rawAngle);
    }
    // 'save' within lid axis: commit current rawAngle to current lidPos field
    else if (cmd == "set") {
        if (_axis == CalibAxis::LID_TOP || _axis == CalibAxis::LID_BOT) {
            _getCurrentAngleRef() = _rawAngle;
            Serial.printf("[Cal] Saved %d deg to %s %s\n",
                          _rawAngle,
                          _axis == CalibAxis::LID_TOP ? "TOP" : "BOT",
                          _lidPos == LidPos::CLOSED   ? "CLOSED" :
                          _lidPos == LidPos::REST      ? "REST"   : "MAX_OPEN");
            _changed = true;
            _printStatus();
        } else {
            Serial.println("[Cal] 'set' only applies to lid axes (top/bot)");
        }
    }

    // -- Preview both lids at named position -----------------------------------
    else if (cmd == "show closed")  { _writeLidsAt(LidPos::CLOSED);   Serial.println("[Cal] Preview: CLOSED"); }
    else if (cmd == "show rest")    { _writeLidsAt(LidPos::REST);      Serial.println("[Cal] Preview: REST"); }
    else if (cmd == "show maxopen") { _writeLidsAt(LidPos::MAX_OPEN);  Serial.println("[Cal] Preview: MAX OPEN"); }

    // -- Tests -----------------------------------------------------------------
    else if (cmd == "test")   _testMotion();
    else if (cmd == "arouse") _testArouse();
    else if (cmd == "blink") {
        _writeLidsAt(LidPos::CLOSED); delay(110);
        _writeLidsAt(LidPos::REST);
        Serial.println("[Cal] Blink");
    }

    // -- Store ------------------------------------------------------------------
    else if (cmd == "save")   { save(); _changed = false; }
    else if (cmd == "load")   { begin(); _printStatus(); }
    else if (cmd == "reset")  { reset(); _printStatus(); }
    else if (cmd == "status") _printStatus();
    else if (cmd == "help")   _printHelp();
    else if (cmd.length() > 0)
        Serial.printf("[Cal] Unknown: '%s' -- type 'help'\n", cmd.c_str());
}

// -- run -- blocking calibration loop ------------------------------------------
bool Calibration::run() {
    _printHelp();
    _changed = false;

    // Start at safe positions
    servoBoard.setServoAngle(SERVO_CH_PAN,  work.panCenter);
    servoBoard.setServoAngle(SERVO_CH_TILT, work.tiltCenter);
    _writeLidsAt(LidPos::REST);
    _axis       = CalibAxis::PAN;
    _panTiltPos = PanTiltPos::CENTER;
    _rawAngle   = work.panCenter;

    String buf = "";
    bool exitWarned = false;

    while (true) {
        if (Serial.available()) {
            char c = Serial.read();
            if (c == '\r') continue;
            if (c == '\n') {
                buf.trim();
                buf.toLowerCase();

                if (buf == "exit") {
                    if (_changed && !exitWarned) {
                        Serial.println("[Cal] Unsaved changes! Type 'save' then 'exit', or 'exit' again to discard");
                        exitWarned = true;
                    } else {
                        break;
                    }
                } else {
                    exitWarned = false;
                    _handleCommand(buf);
                }
                buf = "";
            } else if (buf.length() < 40) {
                buf += c;
            }
        }
        delay(10);
    }

    apply();
    Serial.println("[Cal] Exiting calibration");
    return _changed;
}