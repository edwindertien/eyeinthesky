#pragma once
#include <Arduino.h>
#include "esp_camera.h"
#include "blob_tracker.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════════════════
//  vision.h — camera init + Core 0 vision task
//
//  Supports two pipelines switchable at runtime:
//    VisionMode::BLOBS     — BlobTracker (motion only, person tracking)
//    VisionMode::SALIENCY  — SaliencyMap (colour/contrast/motion weighted)
//
//  Both pipelines output to blobQueue as a BlobResult.
//  Saliency mode produces a single-blob result from the saliency winner.
//
//  Switch with: setVisionMode(VisionMode::SALIENCY)
//  Serial commands: 'sal' / 'blob'
// ═══════════════════════════════════════════════════════════════════════════

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

// Current active pipeline — readable from any task
extern volatile VisionMode activeVisionMode;

bool cameraInit();
void visionTaskStart();
bool visionGetDebug(VisionDebug& out);

// Switch pipeline (thread-safe — vision task reads this each frame)
void setVisionMode(VisionMode mode);