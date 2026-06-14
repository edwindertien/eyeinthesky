#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "blob_tracker.h"
#include "config.h"

// ===========================================================================
//  vision.h -- camera init + Core 0 vision task
//
//  Supports two pipelines switchable at runtime:
//    VisionMode::BLOBS     -- BlobTracker (motion only, person tracking)
//    VisionMode::SALIENCY  -- SaliencyMap (colour/contrast/motion weighted)
//
//  Both pipelines output to blobQueue as a BlobResult.
//  Saliency mode produces a single-blob result from the saliency winner.
//
//  Switch with: setVisionMode(VisionMode::SALIENCY)
//  Serial commands: 'sal' / 'blob'
// ===========================================================================

struct VisionDebug {
    uint8_t  motion[DBG_W * DBG_H];   // motion / saliency map thumbnail
    uint8_t  blobs[DBG_W * DBG_H];    // blob boxes / saliency peak marker
    int      blobCount;
    float    fps;
    uint32_t frameCount;
    VisionMode mode;                   // which pipeline is active
};

extern QueueHandle_t      blobQueue;
extern SemaphoreHandle_t  debugMutex;
extern VisionDebug        visionDebug;

// Current active pipeline -- readable from any task
extern volatile VisionMode activeVisionMode;

bool cameraInit();
void visionTaskStart();
bool visionGetDebug(VisionDebug& out);

// Switch pipeline (thread-safe -- vision task reads this each frame)
void setVisionMode(VisionMode mode);

// Schedule a background reset after a delay (ms).
// Safe to call from Core 1 -- vision task executes it on Core 0.
// If called again before the delay expires, resets the timer.
void scheduleBackgroundReset(uint32_t delayMs);

// Signal from vision task: background has just been reset and is settling.
// Behaviour layer polls this to know when it's safe to start tracking.
extern volatile bool visionBgSettled;     // true once bg model is stable
extern volatile uint32_t visionBgResetMs; // millis() of last bg reset