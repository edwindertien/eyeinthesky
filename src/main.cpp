#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "states.h"
#include "m5servo8.h"
#include "servo_eye.h"
#include "blob_tracker.h"
#include "vision.h"
#include "behaviour.h"
#include "calibration.h"
#include "web_ui.h"
#include <WiFi.h>
#include "saliency.h"

// ===========================================================================
//  main.cpp -- EyeWatcher Stage 2a (blob tracker edition)
//
//  Serial commands:
//    info          -- status dump
//    scan          -- I2C bus scan
//    ansi          -- toggle ANSI live display (use pio device monitor)
//    blobs         -- print current blob list
//    cal           -- enter servo calibration mode
//    bgt <n>       -- set motion threshold (e.g. "bgt 20")
//    bgr <n>       -- set bg learning rate x1000 (e.g. "bgr 5")
//    minblob <n>   -- min blob size in pixels
//    dwell <n>     -- override dwell max ms
//    bgreset       -- force background reset
//    blink / home / reboot
// ===========================================================================

BehaviourConfig behaviour;

// Camera-to-servo mapping -- tune with serial commands or edit defaults here
float camPanScale   = 1.0f;   // reduce for wide-angle (e.g. 0.5 for 120deg FOV)
float camTiltScale  = 1.0f;
float camPanOffset  = 0.0f;   // positive = camera sees scene shifted right
float camTiltOffset = 0.0f;   // positive = camera mounted above eye center
bool  camMirrorX    = false;
bool  camMirrorY    = false;

static bool     ansiEnabled = false;
static uint32_t lastAnsi    = 0;
static uint32_t lastReport  = 0;

// -- ANSI helpers --------------------------------------------------------------
#define ANSI_HOME  "\033[H"
#define ANSI_CLEAR "\033[2J\033[H"
#define ANSI_BOLD  "\033[1m"
#define ANSI_RESET "\033[0m"
#define ANSI_RED   "\033[31m"
#define ANSI_YEL   "\033[33m"
#define ANSI_GRN   "\033[32m"
#define ANSI_CYN   "\033[36m"
#define ANSI_WHT   "\033[37m"

static const char GREY_RAMP[] = " .:-=+*#%@";
#define RAMP_LEN 10

static void renderAnsi(const VisionDebug& dbg) {
    Serial.print(ANSI_HOME);

    // Header
    const char* modeStr = (dbg.mode == VisionMode::BLOBS) ? "BLOB" : "SAL";
    Serial.printf(ANSI_BOLD "EyeWatcher " ANSI_RESET
                  "[" ANSI_YEL "%s" ANSI_RESET "] "
                  "state=" ANSI_CYN "%s" ANSI_RESET
                  " fps=%.0f targets=" ANSI_YEL "%d" ANSI_RESET
                  " pan=%.0f tilt=%.0f arousal=%.2f\n",
                  modeStr, stateName(behaviour_sm.getState()),
                  dbg.fps, dbg.blobCount,
                  eye.getPanDeg(), eye.getTiltDeg(), eye.getArousal());
    Serial.printf("thr=%d bgr=%d minBlob=%d minAge=%d avg=%d timeout=%lums\n",
                  blobTracker.motionThreshold,
                  blobTracker.bgAlphaInt,
                  blobTracker.minBlobPixels,
                  blobTracker.minBlobAge,
                  blobTracker.frameAvgCount,
                  (unsigned long)blobTracker.blobTimeoutMs);

    // Column headers -- change label based on mode
    const char* leftLabel  = (dbg.mode == VisionMode::BLOBS) ? "[ MOTION MASK ]" : "[ SALIENCY MAP ]";
    const char* rightLabel = (dbg.mode == VisionMode::BLOBS) ? "[ BLOBS ]"       : "[ SALIENCY PEAK ]";
    char hdr[DBG_W*2+8];
    snprintf(hdr, sizeof(hdr), "%-*s   %-*s", DBG_W, leftLabel, DBG_W, rightLabel);
    Serial.println(hdr);

    // Side by side: motion mask | blob map
    for (int y = 0; y < DBG_H; y++) {
        Serial.print(ANSI_WHT);
        for (int x = 0; x < DBG_W; x++) {
            uint8_t mv = dbg.motion[y*DBG_W+x];
            if (mv > 180)     Serial.print(ANSI_RED);
            else if (mv > 60) Serial.print(ANSI_YEL);
            else              Serial.print(ANSI_WHT);
            Serial.print(GREY_RAMP[mv * (RAMP_LEN-1) / 255]);
        }
        Serial.print(ANSI_RESET "   ");

        for (int x = 0; x < DBG_W; x++) {
            uint8_t bv = dbg.blobs[y*DBG_W+x];
            if (bv == 255)    Serial.print(ANSI_RED "+");
            else if (bv > 0)  Serial.printf(ANSI_GRN "%c", '0' + (bv/40 % 9));
            else              Serial.print(ANSI_WHT ".");
        }
        Serial.println(ANSI_RESET);
    }
    Serial.printf("\nthr=%d  bgr=%d  minBlob=%d  | ansi bgt bgr minblob dwell bgreset info\n",
                  blobTracker.motionThreshold,
                  blobTracker.bgAlphaInt,
                  blobTracker.minBlobPixels);
}

static void i2cScan() {
    Serial.printf("\n[I2C] SDA=GPIO%d SCL=GPIO%d\n", I2C_SDA, I2C_SCL);
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  0x%02X %s\n", addr, addr==0x25 ? "← M5Servo8" : "");
            found++;
        }
    }
    Serial.printf("  %d found\n\n", found);
}

static void printInfo() {
    Serial.println("\n╔======================================╗");
    Serial.println("║   EyeWatcher -- Blob Tracker          ║");
    Serial.println("╚======================================╝");
    Serial.printf("  State      : %s\n", stateName(behaviour_sm.getState()));
    Serial.printf("  Pan/Tilt   : %.1f / %.1f\n", eye.getPanDeg(), eye.getTiltDeg());
    Serial.printf("  Arousal    : %.2f\n", eye.getArousal());
    Serial.printf("  PSRAM free : %u\n", ESP.getFreePsram());
    Serial.println("\n  Blob tracker settings:");
    Serial.printf("    motionThreshold : %d\n", blobTracker.motionThreshold);
    Serial.printf("    bgAlphaInt      : %d (x0.001)\n", blobTracker.bgAlphaInt);
    Serial.printf("    minBlobPixels   : %d\n", blobTracker.minBlobPixels);
    Serial.printf("    maxBlobPixels   : %d\n", blobTracker.maxBlobPixels);
    Serial.printf("    matchRadius     : %.2f\n", blobTracker.matchRadius);
    Serial.printf("    blobTimeoutMs   : %lu\n", (unsigned long)blobTracker.blobTimeoutMs);
    Serial.printf("\n  Vision mode : %s  (type 'sal' or 'blob' to switch)\n",
                  activeVisionMode == VisionMode::BLOBS ? "BLOBS" : "SALIENCY");
    if (activeVisionMode == VisionMode::SALIENCY) {
        Serial.printf("    wMotion=%.2f wColor=%.2f wBright=%.2f wHabit=%.2f\n",
                      saliency.wMotion, saliency.wColor,
                      saliency.wBright, saliency.wHabit);
        Serial.printf("    habitStr=%.3f habitDec=%.4f threshold=%.2f\n",
                      saliency.habitStrength, saliency.habitDecay, saliency.threshold);
    }
    Serial.println("\n  Commands: info scan ansi mode sal blob bgreset cal");
    Serial.println("            bgt bgr minblob minage avg shift");
    Serial.println("            w.motion w.color w.bright w.habit");
    Serial.println("            habit.str habit.dec threshold");
    Serial.println("            blink home reboot");
}

static void printBlobs() {
    BlobResult blobs = {};
    if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);
    Serial.printf("[Blobs] count=%d  anyMotion=%d\n", blobs.count, blobs.anyMotion);
    for (int i = 0; i < blobs.count; i++) {
        const Blob& b = blobs.blobs[i];
        Serial.printf("  [%d] id=%d  pos=%.2f,%.2f  area=%.3f  "
                      "score=%.2f  age=%lu  vel=%.2f,%.2f\n",
                      i, b.id, b.normX, b.normY, b.area,
                      b.score, (unsigned long)b.age, b.vx, b.vy);
    }
}

// processCommand -- shared by serial and WebSocket handlers
void processCommand(const String& raw) {
    String cmdBuf = raw;
    cmdBuf.trim(); cmdBuf.toLowerCase();
    if (!ansiEnabled) Serial.printf("\n> %s\n", cmdBuf.c_str());

            if      (cmdBuf == "info" || cmdBuf == "status") {
                printInfo();
                // Also print WiFi AP status
                Serial.printf("  WiFi AP   : %s  IP: %s  clients: %d\n",
                              WiFi.softAPSSID().c_str(),
                              WiFi.softAPIP().toString().c_str(),
                              WiFi.softAPgetStationNum());
                Serial.printf("  BgSettled : %s\n",
                              visionBgSettled ? "yes" : "no");
                Serial.printf("  PSRAM free: %u  Heap free: %u\n",
                              ESP.getFreePsram(), ESP.getFreeHeap());
                Serial.printf("  Uptime    : %lus\n",
                              (unsigned long)(millis()/1000));
            }
            else if (cmdBuf == "scan")    i2cScan();

            else if (cmdBuf == "blink")   eye.blink();
            else if (cmdBuf == "home")    eye.setGazeDeg(PAN_CENTER, TILT_CENTER, 0.2f);
            else if (cmdBuf == "reboot")  ESP.restart();
            else if (cmdBuf == "bgreset") blobTracker.resetBackground();
            else if (cmdBuf == "sal" || cmdBuf == "saliency") {
                setVisionMode(VisionMode::SALIENCY);
                Serial.println("[CMD] Switched to SALIENCY mode");
                Serial.println("      Commands: w.motion w.color w.bright w.habit");
                Serial.println("      habitat.str habit.dec threshold");
            }
            else if (cmdBuf == "blob" || cmdBuf == "blobs") {
                if (cmdBuf == "blob") {
                    setVisionMode(VisionMode::BLOBS);
                    Serial.println("[CMD] Switched to BLOB mode");
                } else {
                    // 'blobs' prints blob list (existing command)
                    printBlobs();
                }
            }
            else if (cmdBuf == "mode") {
                Serial.printf("[Mode] %s\n",
                    activeVisionMode == VisionMode::BLOBS ? "BLOBS" : "SALIENCY");
            }
            // Saliency tuning commands (active in SALIENCY mode)
            else if (cmdBuf.startsWith("w.motion "))
                saliency.wMotion = cmdBuf.substring(9).toFloat();
            else if (cmdBuf.startsWith("w.color "))
                saliency.wColor = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("w.bright "))
                saliency.wBright = cmdBuf.substring(9).toFloat();
            else if (cmdBuf.startsWith("w.habit "))
                saliency.wHabit = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("habit.str "))
                saliency.habitStrength = cmdBuf.substring(10).toFloat();
            else if (cmdBuf.startsWith("habit.dec "))
                saliency.habitDecay = cmdBuf.substring(10).toFloat();
            else if (cmdBuf.startsWith("threshold "))
                saliency.threshold = cmdBuf.substring(10).toFloat();
            else if (cmdBuf == "ansi") {
                ansiEnabled = !ansiEnabled;
                if (ansiEnabled) Serial.print(ANSI_CLEAR);
                else             Serial.println("[ANSI off]");
            }
            else if (cmdBuf == "cal") {
                calibration.run();
                eye.begin();
            }
            else if (cmdBuf.startsWith("bgt "))
                blobTracker.motionThreshold = cmdBuf.substring(4).toInt();
            else if (cmdBuf.startsWith("bgr "))
                blobTracker.bgAlphaInt = cmdBuf.substring(4).toInt();
            else if (cmdBuf.startsWith("minblob "))
                blobTracker.minBlobPixels = cmdBuf.substring(8).toInt();
            else if (cmdBuf.startsWith("minage "))
                blobTracker.minBlobAge = cmdBuf.substring(7).toInt();
            else if (cmdBuf.startsWith("avg "))
                blobTracker.frameAvgCount = cmdBuf.substring(4).toInt();
            else if (cmdBuf.startsWith("shift ")) {
                blobTracker.shiftThreshold = cmdBuf.substring(6).toFloat();
                Serial.printf("[Cmd] shiftThreshold = %.2f\n", blobTracker.shiftThreshold);
            }
            else if (cmdBuf == "shiftinfo") {
                Serial.printf("[Shift] threshold=%.2f (lower=more sensitive)\n",
                    blobTracker.shiftThreshold);
                Serial.println("[Shift] type 'shift 0.4' to adjust");
            }
            else if (cmdBuf.startsWith("bgint ")) {
                blobTracker.bgResetIntervalMs = (uint32_t)(cmdBuf.substring(6).toInt() * 1000);
                Serial.printf("[CMD] bgResetInterval = %lus\n",
                    (unsigned long)(blobTracker.bgResetIntervalMs/1000));
            }
            else if (cmdBuf.startsWith("dwell ")) {
                // Override dwell max at runtime
                Serial.println("[CMD] Use habit.str for dwell control");
            }
            else if (cmdBuf.startsWith("pan "))
                eye.setGazeDeg(cmdBuf.substring(4).toFloat(), eye.getTiltDeg(), 0.2f);
            else if (cmdBuf.startsWith("tilt "))
                eye.setGazeDeg(eye.getPanDeg(), cmdBuf.substring(5).toFloat(), 0.2f);
            // Camera mapping tuning -- no recompile needed
            else if (cmdBuf == "caminfo") {
                Serial.printf("[Cam] panScale=%.2f  tiltScale=%.2f\n",
                              camPanScale, camTiltScale);
                Serial.printf("[Cam] panOffset=%.2f  tiltOffset=%.2f\n",
                              camPanOffset, camTiltOffset);
                Serial.printf("[Cam] mirrorX=%s  mirrorY=%s\n",
                              camMirrorX ? "yes" : "no",
                              camMirrorY ? "yes" : "no");
                Serial.println("[Cam] Tune: campan <f>  camtilt <f>  camoffx <f>  camoffy <f>  camflipx  camflipy");
            }
            else if (cmdBuf.startsWith("campan "))   camPanScale   = cmdBuf.substring(7).toFloat();
            else if (cmdBuf.startsWith("camtilt "))  camTiltScale  = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("camoffx "))  camPanOffset  = cmdBuf.substring(8).toFloat();
            else if (cmdBuf.startsWith("camoffy "))  camTiltOffset = cmdBuf.substring(8).toFloat();
            else if (cmdBuf == "camflipx") { camMirrorX = !camMirrorX; Serial.printf("[Cam] mirrorX=%s\n", camMirrorX?"on":"off"); }
            else if (cmdBuf == "camflipy") { camMirrorY = !camMirrorY; Serial.printf("[Cam] mirrorY=%s\n", camMirrorY?"on":"off"); }
            else if (cmdBuf.length() > 0)
                Serial.printf("Unknown: '%s' -- type 'info'\n", cmdBuf.c_str());

}

static String _serialBuf = "";
static void handleSerial() {
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\r') continue;
        if (ch == '\n') {
            processCommand(_serialBuf);
            _serialBuf = "";
        } else if (_serialBuf.length() < 40) {
            _serialBuf += ch;
        }
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t t = millis();
    while (!Serial && millis()-t < 2000) delay(10);

    Serial.println("\n╔======================================╗");
    Serial.println("║   EyeWatcher -- Blob Tracker Edition  ║");
    Serial.println("╚======================================╝");
    Serial.printf("  PSRAM: %u  Heap: %u\n\n", ESP.getPsramSize(), ESP.getFreeHeap());

    // Camera first -- before Wire to avoid I2C peripheral conflict
    if (!blobTracker.begin()) {
        Serial.println("[FATAL] BlobTracker PSRAM alloc failed");
        while(true) delay(1000);
    }

    bool cameraOk = cameraInit();
    if (!cameraOk)
        Serial.println("[WARN] Camera failed -- servo-only mode");

    // I2C + servos after camera
    Wire.begin(I2C_SDA, I2C_SCL, I2C_FREQ);
    delay(50);
    i2cScan();
    calibration.begin();
    if (!eye.begin())
        Serial.println("[WARN] Servo board not found");

    if (cameraOk) {
        visionTaskStart();
        delay(200);
    }

    behaviour_sm.begin();
    webUiBegin();
    lastReport = millis();
    Serial.println("[Boot] Ready -- 'ansi' for live display  'info' for status");
}

void loop() {
    uint32_t now = millis();
    handleSerial();
    webUiLoop();
    behaviour_sm.update();
    eye.update();

    if (ansiEnabled && now - lastAnsi >= 125) {
        lastAnsi = now;
        VisionDebug dbg;
        if (visionGetDebug(dbg)) renderAnsi(dbg);
    }

    if (now - lastReport >= 3000) {
        lastReport = now;
        BlobResult blobs = {};
        if (blobQueue) xQueuePeek(blobQueue, &blobs, 0);
        char logbuf[80];
        snprintf(logbuf, sizeof(logbuf), "[Eye] %s pan=%.0f tilt=%.0f ar=%.2f blobs=%d",
                 stateName(behaviour_sm.getState()),
                 eye.getPanDeg(), eye.getTiltDeg(), eye.getArousal(), blobs.count);
        webUiLog(logbuf);
        if (!ansiEnabled)
            Serial.printf("%s  %umA\n", logbuf, servoBoard.readCurrentMA());
    }

    delay(20);
}