#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "states.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  vision.h — camera initialisation + Core 0 vision task
//
//  Architecture:
//    visionTaskStart() launches a FreeRTOS task pinned to Core 0.
//    Every frame it posts a DetectionResult to detectionQueue (depth 1,
//    overwrite) so Core 1 behaviour always gets the freshest result.
//
//  The task also maintains:
//    debugFrame[]  — 8-bit grayscale thumbnail for ANSI display
//    debugSaliency[] — 8-bit saliency map thumbnail
//    debugTarget   — winning target in thumbnail coords
//  All protected by debugMutex. Read with visionGetDebug().
// ═══════════════════════════════════════════════════════════════════════════

// Thumbnail resolution for ANSI display and saliency map
// QVGA (320×240) downsampled by factor 8 → 40×30 chars
#define DBG_W   40
#define DBG_H   30

struct VisionDebug {
    uint8_t  grey[DBG_W * DBG_H];      // downsampled greyscale
    uint8_t  saliency[DBG_W * DBG_H];  // saliency map
    uint8_t  motion[DBG_W * DBG_H];    // motion channel
    int      targetX;                   // winning target in thumbnail coords
    int      targetY;
    float    targetConf;
    uint32_t frameCount;
    uint32_t fps;                       // measured frames/sec on Core 0
};

// ── Shared queue and mutex ────────────────────────────────────────────────────
extern QueueHandle_t      detectionQueue;   // DetectionResult, depth 1
extern SemaphoreHandle_t  debugMutex;
extern VisionDebug        visionDebug;

// ── Public API ────────────────────────────────────────────────────────────────
bool cameraInit();
void visionTaskStart();

// Safe copy of debug frame — call from Core 1
bool visionGetDebug(VisionDebug& out);