#pragma once
// ═══════════════════════════════════════════════════════════════════════════
//  config.h — EyeWatcher central configuration
//
//  All hardware constants live here (never scattered in .cpp files).
//  Runtime-tunable parameters are in SaliencyConfig / BehaviourConfig structs
//  which get serialised to/from NVS by the web server.
// ═══════════════════════════════════════════════════════════════════════════

// ── WiFi AP ──────────────────────────────────────────────────────────────────
#define WIFI_AP_SSID        "EyeWatcher"
#define WIFI_AP_PASSWORD    "watching1"   // min 8 chars for WPA2; set "" for open
#define WIFI_AP_CHANNEL     6
#define WIFI_AP_MAX_CLIENTS 4

// ── UDP flock broadcast ───────────────────────────────────────────────────────
// All units on the same network; multicast keeps it zero-config
#define UDP_MULTICAST_IP    "239.1.2.3"
#define UDP_MULTICAST_PORT  5555
#define UNIT_ID             1             // Change per physical unit 1–255

// ── I2C — XIAO Grove connector ───────────────────────────────────────────────
// Grove on the Seeed XIAO shield maps to GPIO5 (SDA) and GPIO6 (SCL)
#define I2C_SDA             5
#define I2C_SCL             6
#define I2C_FREQ            400000        // 400 kHz fast-mode; M5 STM32 supports it

// ── M5Stack 8Servo Unit (STM32F030F4, I2C 0x25) ──────────────────────────────
#define M5SERVO8_ADDR       0x25

// Register base addresses (each channel occupies one byte for angle mode)
#define M5S8_REG_ANGLE_BASE     0x00  // Ch1=0x00 … Ch8=0x07  (0–180 degrees)
#define M5S8_REG_PW_BASE        0x10  // Ch1=0x10/0x11 … pulse width low/high (µs)
#define M5S8_REG_SERVO_ENABLE   0x30  // Bit per channel — 0xFF = all on
#define M5S8_REG_MOTOR_POWER    0x31  // 0x01 = MOS on (servo rail powered)
#define M5S8_REG_CURRENT_L      0x40  // INA199 total current, low byte
#define M5S8_REG_CURRENT_H      0x41  // INA199 total current, high byte

// ── Servo channel assignments on the 8Servo unit ────────────────────────────
// Channels are 0-indexed in the driver, matching register offsets
#define SERVO_CH_PAN        0         // Eyeball horizontal (left / right)
#define SERVO_CH_TILT       1         // Eyeball vertical   (up / down)
#define SERVO_CH_LID_TOP    2         // Upper eyelid
#define SERVO_CH_LID_BOT    3         // Lower eyelid

// ── SG90 mechanical limits (degrees) ────────────────────────────────────────
// Constrain to safe range — prevents grinding on hard stops.
// Tune these to match your actual eye mechanism geometry.
#define PAN_CENTER          90
#define PAN_MIN             50
#define PAN_MAX             130

#define TILT_CENTER         85
#define TILT_MIN            65
#define TILT_MAX            105

// Eyelid angles — adjust to suit your 3D-printed mechanism
// "Open" and "closed" may need swapping depending on linkage direction
#define LID_TOP_OPEN        60        // Upper lid fully raised
#define LID_TOP_CLOSED      100       // Upper lid fully lowered
#define LID_TOP_HALF        80        // Drowsy / dozing position

#define LID_BOT_OPEN        110       // Lower lid fully lowered
#define LID_BOT_CLOSED      75        // Lower lid fully raised
#define LID_BOT_HALF        95        // Drowsy / dozing position

// ── Camera (XIAO ESP32-S3 Sense, OV2640/OV3660) ─────────────────────────────
// Pin map is fixed by hardware — do not change
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
// QVGA (320×240) is a good balance: fast enough for saliency, enough resolution
// to locate a person in a wide-angle shot. Drop to QQVGA if CPU headroom is tight.
#define CAM_FRAME_SIZE      FRAMESIZE_QVGA
#define CAM_JPEG_QUALITY    15          // 0=best quality/slowest, 63=worst/fastest

// ═══════════════════════════════════════════════════════════════════════════
//  Runtime-tunable structs
//  Defaults here are conservative / robust starting points.
//  The web UI exposes all fields; values are persisted to NVS.
// ═══════════════════════════════════════════════════════════════════════════

struct SaliencyConfig {
    // ── Background model ────────────────────────────────────────────────────
    // Low learning rate = slow adaptation = robust to gradual light changes
    // High = fast adapt = ignores slow-moving objects
    float   bgLearningRate   = 0.004f;

    // Minimum contour area in pixels to count as a detection (noise rejection)
    int     minContourArea   = 600;

    // Morphological dilation iterations on the motion mask (fills holes)
    int     dilateIterations = 2;

    // ── Detection thresholds ────────────────────────────────────────────────
    // Fraction of frame that must show motion to trigger detection (0.0–1.0)
    float   motionFraction   = 0.005f;

    // Consecutive frames needed to confirm a detection (debounce)
    int     confirmFrames    = 4;

    // Consecutive frames without motion before declaring target lost
    int     lostFrames       = 25;

    // ── Target smoothing ────────────────────────────────────────────────────
    // EMA factor for saliency centroid position (lower = smoother but laggier)
    float   targetSmooth     = 0.2f;

    // Dead zone radius in normalised coords (0.0–1.0); no servo move within
    float   deadZoneNorm     = 0.04f;

    // ── Exposure normalisation ───────────────────────────────────────────────
    // CLAHE equalisation helps with mixed / variable gallery lighting
    bool    useClahe         = true;
    int     claheClipLimit   = 2;
    int     claheTileSize    = 8;
};

struct BehaviourConfig {
    // ── State transition timings (milliseconds) ──────────────────────────────
    uint32_t idleToDozeMs    = 12000;   // No stimulus → start dozing
    uint32_t dozeToSleepMs   = 40000;   // Dozing → fully asleep
    uint32_t wakeDelayMs     = 300;     // Detection → start tracking (reaction)

    // ── Blink timing ────────────────────────────────────────────────────────
    uint32_t blinkIntervalMs = 3500;    // Mean time between blinks
    uint32_t blinkJitterMs   = 1500;    // ± random jitter added to interval
    uint32_t blinkDurationMs = 110;     // Full close→open cycle duration

    // ── Idle scan motion ────────────────────────────────────────────────────
    // Lazy sinusoidal drift when no target — makes Level 1 feel alive
    float    scanPanAmp      = 18.0f;   // degrees peak-to-peak
    float    scanTiltAmp     = 6.0f;
    float    scanPeriodMs    = 9000.0f; // Full scan cycle duration

    // ── Gaze servo speed ────────────────────────────────────────────────────
    // EMA alpha for servo position updates — lower = smoother / slower
    float    gazeAlphaIdle   = 0.04f;   // Lazy drift
    float    gazeAlphaTrack  = 0.14f;   // Tracking a target
    float    gazeAlphaFocus  = 0.06f;   // Locked on — subtle micro-movement

    // ── Flock (network advisory) ─────────────────────────────────────────────
    // How much a peer detection biases this unit's gaze target (0=ignore, 1=copy)
    float    flockInfluence  = 0.25f;
    uint32_t flockTimeoutMs  = 2500;    // Discard peer hint after this long
};

// Global instances — defined in main.cpp, used everywhere via extern
extern SaliencyConfig  saliency;
extern BehaviourConfig behaviour;
