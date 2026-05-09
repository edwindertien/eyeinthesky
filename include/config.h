#pragma once

// ── Vision pipeline selection ─────────────────────────────────────────────────
// Change default here, or switch at runtime with 'sal' / 'blob' serial command
enum class VisionMode : uint8_t {
    BLOBS,      // Motion blob tracker — robust, person-tracking, sleep on static
    SALIENCY,   // Saliency map — colour/contrast/motion weighted, habituation
};

#define VISION_DEFAULT_MODE VisionMode::BLOBS

// ═══════════════════════════════════════════════════════════════════════════
//  config.h — EyeWatcher central configuration
//
//  Hardware:
//    Seeed XIAO ESP32-S3 Sense
//    Seeed Grove Shield for XIAO
//    M5Stack 8Servo Unit on Grove I2C connector
//
//  Servo channels (as wired):
//    ch0 = pan  (left/right eyeball)
//    ch1 = tilt (up/down eyeball)
//    ch2 = lower eyelid
//    ch3 = upper eyelid
//
//  I2C: GPIO5 (SDA), GPIO6 (SCL) — confirmed for Grove Shield + ESP32-S3
// ═══════════════════════════════════════════════════════════════════════════

#define WIFI_AP_SSID        "EyeWatcher"
#define WIFI_AP_PASSWORD    "watching1"
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CLIENTS 4

#define UDP_MULTICAST_IP    "239.1.2.3"
#define UDP_MULTICAST_PORT  5555
#define UNIT_ID             1

// ── I2C ──────────────────────────────────────────────────────────────────────
#define I2C_SDA             5
#define I2C_SCL             6
#define I2C_FREQ            100000

// ── M5Stack 8Servo Unit ──────────────────────────────────────────────────────
#define M5SERVO8_ADDR       0x25

// ── Servo channels (as physically wired) ─────────────────────────────────────
#define SERVO_CH_PAN        0         // ch0 — eyeball left/right
#define SERVO_CH_TILT       1         // ch1 — eyeball up/down
#define SERVO_CH_LID_BOT    2         // ch2 — lower eyelid
#define SERVO_CH_LID_TOP    3         // ch3 — upper eyelid

// ── Eyeball limits ────────────────────────────────────────────────────────────
#define PAN_CENTER          90
#define PAN_MIN             58
#define PAN_MAX             122

#define TILT_CENTER         85
#define TILT_MIN            68
#define TILT_MAX            102

// ── Eyelid model ──────────────────────────────────────────────────────────────
// Lids are controlled by a CENTER position (follows tilt) and a GAP (inter-lid
// distance). Top and bottom lids move symmetrically around the center.
//
// top servo angle  = center - gap/2    (upper lid moves up to open)
// bot servo angle  = center + gap/2    (lower lid moves down to open)
//
// Adjust these to your mechanism. Tune LID_CENTER_OPEN to frame the iris.
// If a lid moves the wrong direction, flip its OPEN/CLOSED by swapping the
// sign of its contribution in _lidToDeg() — or swap angles below.

// Resting center when gaze is horizontal (tracks tilt offset from here)
#define LID_CENTER_DEG      85        // degrees — midpoint between lids at rest

// How many degrees each lid moves from center to fully open (half the gap)
// e.g. 15 means top goes to 70°, bottom to 100° when open
#define LID_HALF_GAP_OPEN   15        // degrees — tune to frame your iris

// How many degrees each lid moves from center to fully closed (blink)
#define LID_HALF_GAP_CLOSED 3         // degrees — nearly zero gap = closed

// How much the lid center follows tilt (1.0 = 1:1, 0.5 = half movement)
#define LID_TILT_FOLLOW     0.6f

// How much extra gap opens when panning left/right (foreshortening compensation)
#define LID_PAN_WIDEN       0.3f      // fraction of max pan range added to gap

// Upper lid servo direction: +1 if increasing angle opens lid, -1 if reversed
#define LID_TOP_DIR         (-1)      // your upper lid is inverted
// Lower lid servo direction
#define LID_BOT_DIR         (-1)

// ── Camera (fixed by hardware) ────────────────────────────────────────────────
#define CAM_PIN_PWDN        -1
#define CAM_PIN_RESET       -1
#define CAM_PIN_XCLK        10
#define CAM_PIN_SIOD        40
#define CAM_PIN_SIOC        39
#define CAM_PIN_D7          48
#define CAM_PIN_D6          11
#define CAM_PIN_D5          12
#define CAM_PIN_D4          14
#define CAM_PIN_D3          16
#define CAM_PIN_D2          18
#define CAM_PIN_D1          17
#define CAM_PIN_D0          15
#define CAM_PIN_VSYNC       38
#define CAM_PIN_HREF        47
#define CAM_PIN_PCLK        13
#define CAM_XCLK_FREQ       20000000
#define CAM_FRAME_SIZE      FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY    15


struct BehaviourConfig {
    uint32_t idleToDozeMs    = 8000;   // no target → dozing after 8s
    uint32_t dozeToSleepMs   = 20000;  // dozing → sleep after 20s
    uint32_t wakeDelayMs     = 400;    // slight pause before tracking starts
    uint32_t blinkIntervalMs = 5000;   // slower blink — more natural at rest
    uint32_t blinkJitterMs   = 2000;   // wider jitter = less regular
    uint32_t blinkDurationMs = 110;
    float    scanPanAmp      = 18.0f;
    float    scanTiltAmp     = 6.0f;
    float    scanPeriodMs    = 9000.0f;
    float    gazeAlphaIdle   = 0.03f;   // slow lazy drift during scan
    float    gazeAlphaTrack  = 0.18f;   // base tracking alpha (boosted by distance)
    float    gazeAlphaFocus  = 0.05f;   // very slow settling in focus state
    float    flockInfluence  = 0.25f;
    uint32_t flockTimeoutMs  = 2500;
};

extern BehaviourConfig behaviour;