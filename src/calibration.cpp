#include "calibration.h"
#include "servo_eye.h"
#include "config.h"

Calibration calibration;

static const char* NVS_NS  = "eyecal";
static const char* NVS_KEY = "caldata";

// ── _isValid ──────────────────────────────────────────────────────────────────
bool Calibration::_isValid(const CalibData& d) const {
    auto inRange = [](uint8_t v, uint8_t lo, uint8_t hi) {
        return v >= lo && v <= hi;
    };
    // Pan: min < center < max, all within 20–160°
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

    // Lid angles: all within 10–170°
    auto inLid = [&](uint8_t v) { return inRange(v, 10, 170); };
    if (!inLid(d.lidTopClosed)  || !inLid(d.lidTopRest)    ||
        !inLid(d.lidTopMaxOpen) || !inLid(d.lidBotClosed)  ||
        !inLid(d.lidBotRest)    || !inLid(d.lidBotMaxOpen)) return false;

    // Lid positions must be distinct (at least 2° between each step)
    if (abs((int)d.lidTopClosed  - (int)d.lidTopRest)    < 2) return false;
    if (abs((int)d.lidTopRest    - (int)d.lidTopMaxOpen) < 2) return false;
    if (abs((int)d.lidBotClosed  - (int)d.lidBotRest)    < 2) return false;
    if (abs((int)d.lidBotRest    - (int)d.lidBotMaxOpen) < 2) return false;

    // Dynamics
    if (d.lidTiltFollow < 0.0f || d.lidTiltFollow > 1.5f) return false;
    if (d.lidPanWiden   < 0.0f || d.lidPanWiden   > 1.0f) return false;

    return true;
}

// ── begin ─────────────────────────────────────────────────────────────────────
void Calibration::begin() {
    _prefs.begin(NVS_NS, true);
    size_t len = _prefs.getBytesLength(NVS_KEY);
    bool loaded = false;
    if (len == sizeof(CalibData)) {
        loaded = (_prefs.getBytes(NVS_KEY, &data, sizeof(CalibData))
                  == sizeof(CalibData));
    }
    _prefs.end();

    if (loaded && _isValid(data)) {
        Serial.println("[Cal] Loaded from NVS");
    } else {
        if (loaded) Serial.println("[Cal] NVS invalid — using defaults");
        else        Serial.println("[Cal] No NVS data — using defaults");
        data = CALIB_DEFAULTS;
    }
    apply();
}

// ── save ─────────────────────────────────────────────────────────────────────
bool Calibration::save() {
    _prefs.begin(NVS_NS, false);
    size_t w = _prefs.putBytes(NVS_KEY, &data, sizeof(CalibData));
    _prefs.end();
    bool ok = (w == sizeof(CalibData));
    Serial.printf("[Cal] Save %s\n", ok ? "OK" : "FAILED");
    return ok;
}

// ── reset ─────────────────────────────────────────────────────────────────────
void Calibration::reset() {
    data = CALIB_DEFAULTS;
    _prefs.begin(NVS_NS, false);
    _prefs.clear();
    _prefs.end();
    apply();
    Serial.println("[Cal] Reset to defaults");
}

// ── apply ─────────────────────────────────────────────────────────────────────
void Calibration::apply() {
    Serial.printf("[Cal] pan %d/%d/%d  tilt %d/%d/%d\n",
                  data.panMin, data.panCenter, data.panMax,
                  data.tiltMin, data.tiltCenter, data.tiltMax);
    Serial.printf("[Cal] topLid closed=%d rest=%d maxOpen=%d\n",
                  data.lidTopClosed, data.lidTopRest, data.lidTopMaxOpen);
    Serial.printf("[Cal] botLid closed=%d rest=%d maxOpen=%d\n",
                  data.lidBotClosed, data.lidBotRest, data.lidBotMaxOpen);
}

// ── _getCurrentAngle / Ref ────────────────────────────────────────────────────
uint8_t Calibration::_getCurrentAngle() const {
    switch (_axis) {
        case CalibAxis::PAN:  return data.panCenter;
        case CalibAxis::TILT: return data.tiltCenter;
        case CalibAxis::LID_TOP:
            switch (_lidPos) {
                case LidPos::CLOSED:   return data.lidTopClosed;
                case LidPos::REST:     return data.lidTopRest;
                case LidPos::MAX_OPEN: return data.lidTopMaxOpen;
            }
        case CalibAxis::LID_BOT:
            switch (_lidPos) {
                case LidPos::CLOSED:   return data.lidBotClosed;
                case LidPos::REST:     return data.lidBotRest;
                case LidPos::MAX_OPEN: return data.lidBotMaxOpen;
            }
    }
    return 90;
}

uint8_t& Calibration::_getCurrentAngleRef() {
    switch (_axis) {
        case CalibAxis::PAN:  return data.panCenter;  // placeholder
        case CalibAxis::TILT: return data.tiltCenter;
        case CalibAxis::LID_TOP:
            switch (_lidPos) {
                case LidPos::CLOSED:   return data.lidTopClosed;
                case LidPos::REST:     return data.lidTopRest;
                case LidPos::MAX_OPEN: return data.lidTopMaxOpen;
            }
        case CalibAxis::LID_BOT:
            switch (_lidPos) {
                case LidPos::CLOSED:   return data.lidBotClosed;
                case LidPos::REST:     return data.lidBotRest;
                case LidPos::MAX_OPEN: return data.lidBotMaxOpen;
            }
    }
    return data.panCenter;
}

// ── _writeServo ───────────────────────────────────────────────────────────────
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

// ── _writeLidsAt — write both lids to a named position ───────────────────────
void Calibration::_writeLidsAt(LidPos pos) {
    uint8_t top, bot;
    switch (pos) {
        case LidPos::CLOSED:
            top = data.lidTopClosed;   bot = data.lidBotClosed;   break;
        case LidPos::REST:
            top = data.lidTopRest;     bot = data.lidBotRest;     break;
        case LidPos::MAX_OPEN:
            top = data.lidTopMaxOpen;  bot = data.lidBotMaxOpen;  break;
        default:
            top = data.lidTopRest;     bot = data.lidBotRest;     break;
    }
    servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
    servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
}

// ── _nudge ────────────────────────────────────────────────────────────────────
void Calibration::_nudge(int delta) {
    int next = (int)_rawAngle + delta;
    next = constrain(next, 0, 180);
    _rawAngle = (uint8_t)next;
    _writeServo(_rawAngle);

    // Also save into data so status shows live value
    _getCurrentAngleRef() = _rawAngle;
    _changed = true;
    _printStatus();
}

// ── _printStatus ──────────────────────────────────────────────────────────────
void Calibration::_printStatus() {
    const char* axName =
        _axis == CalibAxis::PAN     ? "PAN" :
        _axis == CalibAxis::TILT    ? "TILT" :
        _axis == CalibAxis::LID_TOP ? "LID TOP" : "LID BOT";
    const char* posName =
        _lidPos == LidPos::CLOSED   ? "CLOSED" :
        _lidPos == LidPos::REST     ? "REST" : "MAX_OPEN";

    Serial.printf("\n[Cal] axis=%-7s  pos=%-8s  angle=%3d°\n",
                  axName, posName, _rawAngle);
    Serial.printf("  Pan : min=%3d  center=%3d  max=%3d\n",
                  data.panMin, data.panCenter, data.panMax);
    Serial.printf("  Tilt: min=%3d  center=%3d  max=%3d\n",
                  data.tiltMin, data.tiltCenter, data.tiltMax);
    Serial.printf("  Top : closed=%3d  rest=%3d  maxOpen=%3d\n",
                  data.lidTopClosed, data.lidTopRest, data.lidTopMaxOpen);
    Serial.printf("  Bot : closed=%3d  rest=%3d  maxOpen=%3d\n",
                  data.lidBotClosed, data.lidBotRest, data.lidBotMaxOpen);
}

// ── _printHelp ────────────────────────────────────────────────────────────────
void Calibration::_printHelp() {
    Serial.println("\n╔══════════════════════════════════════════════╗");
    Serial.println("║          SERVO CALIBRATION MODE             ║");
    Serial.println("╠══════════════════════════════════════════════╣");
    Serial.println("║  Select axis:                                ║");
    Serial.println("║    pan | tilt | top | bot                    ║");
    Serial.println("║  Nudge selected servo:                       ║");
    Serial.println("║    +  -   (±1°)     ++  --   (±5°)          ║");
    Serial.println("║  Pan/tilt limits:                            ║");
    Serial.println("║    center | min | max                        ║");
    Serial.println("║  Lid positions (after selecting top/bot):    ║");
    Serial.println("║    closed   → go to / save closed position   ║");
    Serial.println("║    rest     → go to / save rest position     ║");
    Serial.println("║    maxopen  → go to / save max-open position ║");
    Serial.println("║  Preview both lids together:                 ║");
    Serial.println("║    show closed | show rest | show maxopen    ║");
    Serial.println("║  Tests:                                      ║");
    Serial.println("║    test    → sweep full range of motion      ║");
    Serial.println("║    blink   → test blink animation            ║");
    Serial.println("║    arouse  → test arousal 0→1 sweep          ║");
    Serial.println("║  Store:                                      ║");
    Serial.println("║    save | load | reset | status | exit       ║");
    Serial.println("╚══════════════════════════════════════════════╝");
    _printStatus();
}

// ── _testMotion ───────────────────────────────────────────────────────────────
void Calibration::_testMotion() {
    Serial.println("[Cal] Testing...");

    Serial.println("  Pan sweep");
    for (int a = data.panMin; a <= data.panMax; a += 2)
        { servoBoard.setServoAngle(SERVO_CH_PAN, a); delay(18); }
    for (int a = data.panMax; a >= data.panMin; a -= 2)
        { servoBoard.setServoAngle(SERVO_CH_PAN, a); delay(18); }
    servoBoard.setServoAngle(SERVO_CH_PAN, data.panCenter);
    delay(300);

    Serial.println("  Tilt sweep");
    for (int a = data.tiltMin; a <= data.tiltMax; a += 2)
        { servoBoard.setServoAngle(SERVO_CH_TILT, a); delay(18); }
    for (int a = data.tiltMax; a >= data.tiltMin; a -= 2)
        { servoBoard.setServoAngle(SERVO_CH_TILT, a); delay(18); }
    servoBoard.setServoAngle(SERVO_CH_TILT, data.tiltCenter);
    delay(300);

    Serial.println("  Lids: maxOpen → rest → closed → rest");
    _writeLidsAt(LidPos::MAX_OPEN); delay(600);
    _writeLidsAt(LidPos::REST);     delay(600);
    _writeLidsAt(LidPos::CLOSED);   delay(400);
    _writeLidsAt(LidPos::REST);     delay(400);

    Serial.println("[Cal] Done");
}

// ── _testArouse — sweep arousal 0→1 ──────────────────────────────────────────
void Calibration::_testArouse() {
    Serial.println("[Cal] Arousal sweep: closed → rest → maxOpen → rest → closed");
    // closed → rest
    for (int i = 0; i <= 20; i++) {
        float t = i / 20.0f;
        uint8_t top = data.lidTopClosed + t * (data.lidTopRest - data.lidTopClosed);
        uint8_t bot = data.lidBotClosed + t * (data.lidBotRest - data.lidBotClosed);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // rest → maxOpen
    for (int i = 0; i <= 20; i++) {
        float t = i / 20.0f;
        uint8_t top = data.lidTopRest + t * (data.lidTopMaxOpen - data.lidTopRest);
        uint8_t bot = data.lidBotRest + t * (data.lidBotMaxOpen - data.lidBotRest);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // back to rest
    for (int i = 20; i >= 0; i--) {
        float t = i / 20.0f;
        uint8_t top = data.lidTopRest + t * (data.lidTopMaxOpen - data.lidTopRest);
        uint8_t bot = data.lidBotRest + t * (data.lidBotMaxOpen - data.lidBotRest);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    delay(400);
    // rest → closed
    for (int i = 20; i >= 0; i--) {
        float t = i / 20.0f;
        uint8_t top = data.lidTopClosed + t * (data.lidTopRest - data.lidTopClosed);
        uint8_t bot = data.lidBotClosed + t * (data.lidBotRest - data.lidBotClosed);
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, top);
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, bot);
        delay(40);
    }
    _writeLidsAt(LidPos::REST);
    Serial.println("[Cal] Arouse done");
}

// ── _handleCommand ────────────────────────────────────────────────────────────
void Calibration::_handleCommand(const String& cmd) {

    // ── Axis selection ────────────────────────────────────────────────────────
    if (cmd == "pan") {
        _axis = CalibAxis::PAN;
        _rawAngle = data.panCenter;
        servoBoard.setServoAngle(SERVO_CH_PAN, _rawAngle);
        Serial.println("[Cal] Axis: PAN  (+ - center min max)");
        _printStatus();
    }
    else if (cmd == "tilt") {
        _axis = CalibAxis::TILT;
        _rawAngle = data.tiltCenter;
        servoBoard.setServoAngle(SERVO_CH_TILT, _rawAngle);
        Serial.println("[Cal] Axis: TILT  (+ - center min max)");
        _printStatus();
    }
    else if (cmd == "top") {
        _axis = CalibAxis::LID_TOP;
        _lidPos = LidPos::REST;
        _rawAngle = data.lidTopRest;
        servoBoard.setServoAngle(SERVO_CH_LID_TOP, _rawAngle);
        Serial.println("[Cal] Axis: LID TOP  (+ -)");
        Serial.println("       then: closed | rest | maxopen  to select & save position");
        _printStatus();
    }
    else if (cmd == "bot") {
        _axis = CalibAxis::LID_BOT;
        _lidPos = LidPos::REST;
        _rawAngle = data.lidBotRest;
        servoBoard.setServoAngle(SERVO_CH_LID_BOT, _rawAngle);
        Serial.println("[Cal] Axis: LID BOT  (+ -)");
        Serial.println("       then: closed | rest | maxopen  to select & save position");
        _printStatus();
    }

    // ── Nudge ─────────────────────────────────────────────────────────────────
    else if (cmd == "+")   _nudge(+1);
    else if (cmd == "-")   _nudge(-1);
    else if (cmd == "++")  _nudge(+5);
    else if (cmd == "--")  _nudge(-5);

    // ── Pan/tilt limit setters ────────────────────────────────────────────────
    else if (cmd == "center") {
        if      (_axis == CalibAxis::PAN)  data.panCenter  = _rawAngle;
        else if (_axis == CalibAxis::TILT) data.tiltCenter = _rawAngle;
        Serial.printf("[Cal] Center = %d°\n", _rawAngle);
        _changed = true;
    }
    else if (cmd == "min") {
        if      (_axis == CalibAxis::PAN)  data.panMin  = _rawAngle;
        else if (_axis == CalibAxis::TILT) data.tiltMin = _rawAngle;
        Serial.printf("[Cal] Min = %d°\n", _rawAngle);
        _changed = true;
    }
    else if (cmd == "max") {
        if      (_axis == CalibAxis::PAN)  data.panMax  = _rawAngle;
        else if (_axis == CalibAxis::TILT) data.tiltMax = _rawAngle;
        Serial.printf("[Cal] Max = %d°\n", _rawAngle);
        _changed = true;
    }

    // ── Lid position select + save ────────────────────────────────────────────
    // Typing 'closed' / 'rest' / 'maxopen' does two things:
    //   1. Jumps the servo to the currently stored angle for that position
    //   2. Sets _lidPos so subsequent +/- nudges and 'save' apply there
    else if (cmd == "closed") {
        _lidPos = LidPos::CLOSED;
        _rawAngle = (_axis == CalibAxis::LID_TOP) ? data.lidTopClosed : data.lidBotClosed;
        _writeServo(_rawAngle);
        Serial.printf("[Cal] → CLOSED (%d°)  nudge then type 'save'\n", _rawAngle);
        _printStatus();
    }
    else if (cmd == "rest") {
        _lidPos = LidPos::REST;
        _rawAngle = (_axis == CalibAxis::LID_TOP) ? data.lidTopRest : data.lidBotRest;
        _writeServo(_rawAngle);
        Serial.printf("[Cal] → REST (%d°)  nudge then type 'save'\n", _rawAngle);
        _printStatus();
    }
    else if (cmd == "maxopen") {
        _lidPos = LidPos::MAX_OPEN;
        _rawAngle = (_axis == CalibAxis::LID_TOP) ? data.lidTopMaxOpen : data.lidBotMaxOpen;
        _writeServo(_rawAngle);
        Serial.printf("[Cal] → MAX OPEN (%d°)  nudge then type 'save'\n", _rawAngle);
        _printStatus();
    }
    // 'save' within lid axis: commit current rawAngle to current lidPos field
    else if (cmd == "set") {
        if (_axis == CalibAxis::LID_TOP || _axis == CalibAxis::LID_BOT) {
            _getCurrentAngleRef() = _rawAngle;
            Serial.printf("[Cal] Saved %d° to %s %s\n",
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

    // ── Preview both lids at named position ───────────────────────────────────
    else if (cmd == "show closed")  { _writeLidsAt(LidPos::CLOSED);   Serial.println("[Cal] Preview: CLOSED"); }
    else if (cmd == "show rest")    { _writeLidsAt(LidPos::REST);      Serial.println("[Cal] Preview: REST"); }
    else if (cmd == "show maxopen") { _writeLidsAt(LidPos::MAX_OPEN);  Serial.println("[Cal] Preview: MAX OPEN"); }

    // ── Tests ─────────────────────────────────────────────────────────────────
    else if (cmd == "test")   _testMotion();
    else if (cmd == "arouse") _testArouse();
    else if (cmd == "blink") {
        _writeLidsAt(LidPos::CLOSED); delay(110);
        _writeLidsAt(LidPos::REST);
        Serial.println("[Cal] Blink");
    }

    // ── Store ──────────────────────────────────────────────────────────────────
    else if (cmd == "save")   { save(); _changed = false; }
    else if (cmd == "load")   { begin(); _printStatus(); }
    else if (cmd == "reset")  { reset(); _printStatus(); }
    else if (cmd == "status") _printStatus();
    else if (cmd == "help")   _printHelp();
    else if (cmd.length() > 0)
        Serial.printf("[Cal] Unknown: '%s' — type 'help'\n", cmd.c_str());
}

// ── run — blocking calibration loop ──────────────────────────────────────────
bool Calibration::run() {
    _printHelp();
    _changed = false;

    // Start at safe positions
    servoBoard.setServoAngle(SERVO_CH_PAN,  data.panCenter);
    servoBoard.setServoAngle(SERVO_CH_TILT, data.tiltCenter);
    _writeLidsAt(LidPos::REST);
    _axis     = CalibAxis::PAN;
    _rawAngle = data.panCenter;

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