#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "states.h"
#include "m5servo8.h"
#include "servo_eye.h"
#include "vision.h"
#include "saliency.h"
#include "behaviour.h"

// ═══════════════════════════════════════════════════════════════════════════
//  main.cpp — EyeWatcher Stage 2a
//  Camera + saliency + behaviour state machine + ANSI debug display
//
//  Serial commands (115200) — use 'pio device monitor' for ANSI mode:
//    info          — full status dump
//    scan          — I2C bus scan
//    ansi          — toggle live ANSI display
//    sal           — print saliency weights
//    w.motion <f>  — set motion weight
//    w.color  <f>  — set colour saliency weight
//    w.bright <f>  — set brightness weight
//    w.habit  <f>  — set habituation suppression weight
//    habit.str <f> — habituation stamp strength
//    habit.dec <f> — habituation decay rate (e.g. 0.99)
//    threshold <f> — detection threshold (e.g. 0.15)
//    bglr <n>      — background learning rate ×1000 (e.g. 5 = 0.005)
//    blink / home / reboot
// ═══════════════════════════════════════════════════════════════════════════

BehaviourConfig behaviour;   // used in servo_eye.cpp + behaviour.cpp

// ── ANSI display ─────────────────────────────────────────────────────────────
static bool     ansiEnabled = false;
static uint32_t lastAnsi    = 0;
static uint32_t lastReport  = 0;

#define ANSI_CLEAR  "\033[2J\033[H"
#define ANSI_HOME   "\033[H"
#define ANSI_BOLD   "\033[1m"
#define ANSI_RESET  "\033[0m"
#define ANSI_RED    "\033[31m"
#define ANSI_YEL    "\033[33m"
#define ANSI_GRN    "\033[32m"
#define ANSI_CYN    "\033[36m"
#define ANSI_WHT    "\033[37m"

static const char GREY_RAMP[] = " .:-=+*#%@";
#define RAMP_LEN 10

static void renderAnsi(const VisionDebug& dbg, EyeState state) {
    Serial.print(ANSI_HOME);
    Serial.printf(ANSI_BOLD "EyeWatcher " ANSI_RESET
                  "state=" ANSI_CYN "%s" ANSI_RESET
                  " fps=" ANSI_GRN "%lu" ANSI_RESET
                  " pan=%.0f tilt=%.0f arousal=%.2f\n",
                  stateName(state), dbg.fps,
                  eye.getPanDeg(), eye.getTiltDeg(), eye.getArousal());
    Serial.printf("target=" ANSI_YEL "%.2f,%.2f" ANSI_RESET
                  " conf=" ANSI_YEL "%.2f" ANSI_RESET
                  "  hStr=%.2f hDec=%.3f thr=%.2f\n",
                  dbg.targetX>=0 ? (float)dbg.targetX/DBG_W : -1.f,
                  dbg.targetY>=0 ? (float)dbg.targetY/DBG_H : -1.f,
                  dbg.targetConf,
                  saliency.habitStrength, saliency.habitDecay, saliency.threshold);

    // Column headers
    char hdr[DBG_W*2+8];
    snprintf(hdr, sizeof(hdr), "%-*s   %-*s",
             DBG_W, "[ CAMERA ]", DBG_W, "[ SALIENCY ]");
    Serial.println(hdr);

    for (int y = 0; y < DBG_H; y++) {
        // Camera
        Serial.print(ANSI_WHT);
        for (int x = 0; x < DBG_W; x++) {
            if (dbg.targetX == x && dbg.targetY == y) {
                Serial.print(ANSI_RED "+" ANSI_WHT);
            } else {
                Serial.print(GREY_RAMP[dbg.grey[y*DBG_W+x] * (RAMP_LEN-1) / 255]);
            }
        }
        Serial.print(ANSI_RESET "   ");
        // Saliency
        for (int x = 0; x < DBG_W; x++) {
            uint8_t sv = dbg.saliency[y*DBG_W+x];
            const char* col = sv > 180 ? ANSI_RED : (sv > 80 ? ANSI_YEL : ANSI_WHT);
            Serial.print(col);
            if (dbg.targetX == x && dbg.targetY == y)
                Serial.print(ANSI_RED "+");
            else
                Serial.print(GREY_RAMP[sv * (RAMP_LEN-1) / 255]);
        }
        Serial.println(ANSI_RESET);
    }
    Serial.printf("\nweights: motion=%.2f color=%.2f bright=%.2f habit=%.2f\n",
                  saliency.wMotion, saliency.wColor,
                  saliency.wBright, saliency.wHabit);
    Serial.println("type: ansi | w.motion | w.habit | habit.str | threshold | info");
}

static void i2cScan() {
    Serial.printf("\n[I2C] Scanning SDA=GPIO%d SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X %s\n", addr, addr==0x25?"← M5Servo8":"");
            found++;
        }
    }
    Serial.printf("  %d found\n\n", found);
}

static void printInfo() {
    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║   EyeWatcher  Stage 2a               ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.printf("  State    : %s\n", stateName(behaviour_sm.getState()));
    Serial.printf("  Pan/Tilt : %.1f / %.1f\n", eye.getPanDeg(), eye.getTiltDeg());
    Serial.printf("  Arousal  : %.2f\n", eye.getArousal());
    Serial.printf("  PSRAM    : %u free\n", ESP.getFreePsram());
    Serial.printf("  Heap     : %u free\n", ESP.getFreeHeap());
    Serial.printf("  motion=%.2f color=%.2f bright=%.2f habit=%.2f\n",
                  saliency.wMotion, saliency.wColor,
                  saliency.wBright, saliency.wHabit);
    Serial.printf("  habitStr=%.3f habitDec=%.4f thresh=%.2f bglr=%d\n",
                  saliency.habitStrength, saliency.habitDecay,
                  saliency.threshold, saliency.bgAlphaInt);
}

static String cmdBuf = "";
static void handleSerial() {
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            cmdBuf.trim(); cmdBuf.toLowerCase();
            if (!ansiEnabled) Serial.printf("\n> %s\n", cmdBuf.c_str());

            if      (cmdBuf == "info")    printInfo();
            else if (cmdBuf == "scan")    i2cScan();
            else if (cmdBuf == "ansi") {
                ansiEnabled = !ansiEnabled;
                if (ansiEnabled) Serial.print(ANSI_CLEAR);
                else             Serial.println("[ANSI off]");
            }
            else if (cmdBuf == "sal")
                Serial.printf("motion=%.2f color=%.2f bright=%.2f habit=%.2f\n"
                              "hStr=%.3f hDec=%.4f thresh=%.2f bglr=%d\n",
                              saliency.wMotion,saliency.wColor,
                              saliency.wBright,saliency.wHabit,
                              saliency.habitStrength,saliency.habitDecay,
                              saliency.threshold,saliency.bgAlphaInt);
            else if (cmdBuf == "blink")   eye.blink();
            else if (cmdBuf == "home")    eye.setGazeDeg(PAN_CENTER,TILT_CENTER,0.15f);
            else if (cmdBuf == "reboot")  ESP.restart();
            else if (cmdBuf == "det") {
                // Peek at current detection queue — shows what saliency is reporting
                DetectionResult det = {};
                if (detectionQueue && xQueuePeek(detectionQueue, &det, 0) == pdTRUE) {
                    Serial.printf("[Det] valid=%d normX=%.3f normY=%.3f conf=%.3f age=%lums\n",
                                  det.valid, det.normX, det.normY, det.confidence,
                                  millis() - det.timestamp);
                } else {
                    Serial.println("[Det] Queue empty or not started");
                }
                // Also show vision debug stats
                VisionDebug dbg;
                if (visionGetDebug(dbg)) {
                    Serial.printf("[Vision] fps=%lu frames=%lu targetX=%d targetY=%d conf=%.3f\n",
                                  dbg.fps, dbg.frameCount, dbg.targetX, dbg.targetY, dbg.targetConf);
                }
            }
            else if (cmdBuf == "thresh") {
                Serial.printf("[Sal] threshold=%.3f wMotion=%.2f wHabit=%.2f\n",
                              saliency.threshold, saliency.wMotion, saliency.wHabit);
            }
            else if (cmdBuf.startsWith("w.motion "))  saliency.wMotion       = cmdBuf.substring(9).toFloat();
            else if (cmdBuf.startsWith("w.color "))   saliency.wColor        = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("w.bright "))  saliency.wBright       = cmdBuf.substring(9).toFloat();
            else if (cmdBuf.startsWith("w.habit "))   saliency.wHabit        = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("habit.str ")) saliency.habitStrength = cmdBuf.substring(10).toFloat();
            else if (cmdBuf.startsWith("habit.dec ")) saliency.habitDecay    = cmdBuf.substring(10).toFloat();
            else if (cmdBuf.startsWith("threshold ")) saliency.threshold     = cmdBuf.substring(10).toFloat();
            else if (cmdBuf.startsWith("bglr "))      saliency.bgAlphaInt    = cmdBuf.substring(5).toInt();
            else if (cmdBuf.startsWith("pan "))
                eye.setGazeDeg(cmdBuf.substring(4).toFloat(), eye.getTiltDeg(), 0.2f);
            else if (cmdBuf.startsWith("tilt "))
                eye.setGazeDeg(eye.getPanDeg(), cmdBuf.substring(5).toFloat(), 0.2f);
            else if (cmdBuf.length() > 0)
                Serial.printf("Unknown: '%s'\n", cmdBuf.c_str());

            cmdBuf = "";
        } else {
            if (cmdBuf.length() < 40) cmdBuf += c;
        }
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis()-t < 2000) delay(10);

    Serial.println("\n╔══════════════════════════════════════╗");
    Serial.println("║   EyeWatcher  Stage 2a  — booting   ║");
    Serial.println("╚══════════════════════════════════════╝");
    Serial.printf("  PSRAM: %u  Heap: %u\n", ESP.getPsramSize(), ESP.getFreeHeap());

    // ── Camera FIRST — must init before Wire to avoid I2C peripheral conflict ──
    // The OV3660 SCCB uses GPIO40/39 (I2C peripheral 1).
    // Wire (GPIO5/6, peripheral 0) must not be started first.
    if (!saliency.begin()) {
        Serial.println("[FATAL] Saliency PSRAM alloc failed"); while(true) delay(1000);
    }

    bool cameraOk = cameraInit();
    if (!cameraOk) {
        Serial.println("[WARN] Camera init failed — running servo-only mode");
        Serial.println("[WARN] Check: ribbon cable on XIAO Sense? Board seated?");
    }

    // ── I2C + servos AFTER camera ────────────────────────────────────────────
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(50);
    i2cScan();
    if (!eye.begin())
        Serial.println("[WARN] Servo board not found — vision tuning still works");

    if (cameraOk) {
        // Vision task → Core 0 (extracts grey from JPEG internally)
        visionTaskStart();
        delay(300);
    } else {
        Serial.println("[INFO] Vision task not started — no camera");
    }

    // Behaviour state machine
    behaviour_sm.begin();

    lastReport = millis();
    Serial.println("[Boot] Ready — 'ansi' for live display  'info' for status");
    Serial.println("       Use 'pio device monitor' terminal for correct ANSI rendering");
}

// ── loop Core 1 ───────────────────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    handleSerial();
    behaviour_sm.update();
    eye.update();

    // ANSI live display at 8 fps
    if (ansiEnabled && now - lastAnsi >= 125) {
        lastAnsi = now;
        VisionDebug dbg;
        if (visionGetDebug(dbg)) renderAnsi(dbg, behaviour_sm.getState());
    }

    // Plain report every 3s when ANSI off
    if (!ansiEnabled && now - lastReport >= 3000) {
        lastReport = now;
        Serial.printf("[Eye] %s  pan=%.1f tilt=%.1f arousal=%.2f  %umA\n",
                      stateName(behaviour_sm.getState()),
                      eye.getPanDeg(), eye.getTiltDeg(), eye.getArousal(),
                      servoBoard.readCurrentMA());
    }

    delay(20);
}