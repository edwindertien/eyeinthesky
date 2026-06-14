#include "vision.h"
#include <cmath>
#include "blob_tracker.h"
#include "saliency.h"
#include <esp_camera.h>

// -- Shared state --------------------------------------------------------------
QueueHandle_t       blobQueue        = nullptr;
SemaphoreHandle_t   debugMutex       = nullptr;
VisionDebug         visionDebug      = {};
volatile VisionMode activeVisionMode = VISION_DEFAULT_MODE;

// Scheduled background reset
static volatile uint32_t _scheduledResetMs  = 0;  // millis() to fire, 0=none
static volatile uint32_t _scheduledResetDelay = 0;

// Background settled flag -- set by vision task, read by behaviour
volatile bool     visionBgSettled  = false;
volatile uint32_t visionBgResetMs  = 0;

void scheduleBackgroundReset(uint32_t delayMs) {
    _scheduledResetMs   = millis() + delayMs;
    _scheduledResetDelay = delayMs;
    visionBgSettled     = false;
    Serial.printf("[Vision] Background reset scheduled in %lums\n",
                  (unsigned long)delayMs);
}

void setVisionMode(VisionMode mode) {
    activeVisionMode = mode;
    Serial.printf("[Vision] Mode -> %s\n",
                  mode == VisionMode::BLOBS ? "BLOBS" : "SALIENCY");
    // Reset both pipelines on switch so stale state doesn't carry over
    if (mode == VisionMode::BLOBS) {
        blobTracker.resetBackground();
    } else {
        saliency.resetHabit();
    }
}

// -- Camera init ---------------------------------------------------------------
bool cameraInit() {
    // Detect sensor PID first to choose correct init parameters
    // Quick minimal init to read PID only
    {
        camera_config_t probe = {};
        probe.ledc_channel=LEDC_CHANNEL_0; probe.ledc_timer=LEDC_TIMER_0;
        probe.pin_d0=CAM_PIN_D0; probe.pin_d1=CAM_PIN_D1;
        probe.pin_d2=CAM_PIN_D2; probe.pin_d3=CAM_PIN_D3;
        probe.pin_d4=CAM_PIN_D4; probe.pin_d5=CAM_PIN_D5;
        probe.pin_d6=CAM_PIN_D6; probe.pin_d7=CAM_PIN_D7;
        probe.pin_xclk=CAM_PIN_XCLK; probe.pin_pclk=CAM_PIN_PCLK;
        probe.pin_vsync=CAM_PIN_VSYNC; probe.pin_href=CAM_PIN_HREF;
        probe.pin_sccb_sda=CAM_PIN_SIOD; probe.pin_sccb_scl=CAM_PIN_SIOC;
        probe.pin_pwdn=CAM_PIN_PWDN; probe.pin_reset=CAM_PIN_RESET;
        probe.xclk_freq_hz = 10000000;
        probe.pixel_format = PIXFORMAT_JPEG;
        probe.frame_size   = FRAMESIZE_QQVGA;
        probe.jpeg_quality = 12;
        probe.fb_count     = 1;
        probe.fb_location  = CAMERA_FB_IN_PSRAM;
        probe.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
        esp_camera_init(&probe);
    }
    sensor_t* probe_s = esp_camera_sensor_get();
    uint32_t sensorPID = probe_s ? probe_s->id.PID : 0;
    esp_camera_deinit();
    delay(100);

    // Choose xclk and grab mode based on sensor
    // OV2640: 10MHz + GRAB_WHEN_EMPTY
    // OV3660/OV5640: 20MHz + GRAB_LATEST
    bool isOV2640 = (sensorPID == 0x26);
    uint32_t xclkHz  = isOV2640 ? 10000000 : 20000000;
    camera_grab_mode_t grabMode = isOV2640 ? CAMERA_GRAB_WHEN_EMPTY : CAMERA_GRAB_LATEST;
    Serial.printf("[Camera] Sensor probe: 0x%04X -- using %luMHz %s\n",
                  sensorPID, (unsigned long)(xclkHz/1000000),
                  isOV2640 ? "GRAB_WHEN_EMPTY" : "GRAB_LATEST");

    auto doInit = [&](pixformat_t fmt, framesize_t sz) -> bool {
        camera_config_t cfg = {};
        cfg.ledc_channel = LEDC_CHANNEL_0; cfg.ledc_timer = LEDC_TIMER_0;
        cfg.pin_d0=CAM_PIN_D0; cfg.pin_d1=CAM_PIN_D1;
        cfg.pin_d2=CAM_PIN_D2; cfg.pin_d3=CAM_PIN_D3;
        cfg.pin_d4=CAM_PIN_D4; cfg.pin_d5=CAM_PIN_D5;
        cfg.pin_d6=CAM_PIN_D6; cfg.pin_d7=CAM_PIN_D7;
        cfg.pin_xclk=CAM_PIN_XCLK;  cfg.pin_pclk=CAM_PIN_PCLK;
        cfg.pin_vsync=CAM_PIN_VSYNC; cfg.pin_href=CAM_PIN_HREF;
        cfg.pin_sccb_sda=CAM_PIN_SIOD; cfg.pin_sccb_scl=CAM_PIN_SIOC;
        cfg.pin_pwdn=CAM_PIN_PWDN;   cfg.pin_reset=CAM_PIN_RESET;
        cfg.xclk_freq_hz = xclkHz;
        cfg.pixel_format = fmt;
        cfg.frame_size   = sz;
        cfg.jpeg_quality = 10;
        cfg.fb_count     = (fmt == PIXFORMAT_GRAYSCALE) ? 2 :
                           (isOV2640 ? 1 : (psramFound() ? 2 : 1));
        cfg.fb_location  = CAMERA_FB_IN_PSRAM;
        cfg.grab_mode    = grabMode;
        return esp_camera_init(&cfg) == ESP_OK;
    };

    Serial.println("[Camera] Phase 1: JPEG UXGA...");
    if (!doInit(PIXFORMAT_JPEG, FRAMESIZE_UXGA)) {
        Serial.println("[Camera] JPEG UXGA FAILED"); return false;
    }
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[Camera] JPEG grab FAILED"); return false; }
    Serial.printf("[Camera] JPEG OK -- %dx%d len=%u\n", fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);

    Serial.println("[Camera] Phase 2: GRAYSCALE QVGA...");
    esp_camera_deinit(); delay(100);
    if (!doInit(PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA)) {
        Serial.println("[Camera] GRAYSCALE FAILED"); return false;
    }

    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        const char* snm =
            s->id.PID == 0x3660 ? "OV3660" :
            s->id.PID == 0x26   ? "OV2640" :
            s->id.PID == 0x5640 ? "OV5640" : "unknown";
        Serial.printf("[Camera] Sensor: 0x%04X (%s) VER=0x%02X\n",
                      s->id.PID, snm, s->id.VER);

        if (s->id.PID == 0x3660) {
            // OV3660: physically inverted on XIAO Sense board
            s->set_vflip(s, 1);
            s->set_brightness(s, 1);
            s->set_saturation(s, -2);
        } else if (s->id.PID == 0x26) {
            // OV2640: normal orientation, standard exposure
            s->set_vflip(s, 0);
            s->set_hmirror(s, 0);
            s->set_brightness(s, 0);
            s->set_saturation(s, 0);
            s->set_contrast(s, 1);
        } else if (s->id.PID == 0x5640) {
            // OV5640: same inversion as OV3660 on XIAO Sense
            s->set_vflip(s, 1);
            s->set_hmirror(s, 1);
            s->set_brightness(s, 1);
            s->set_saturation(s, -1);
        }
        // Common settings
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gainceiling(s, (gainceiling_t)4);
        s->set_lenc(s, 1);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
    }
    delay(300);

    fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[Camera] Final grab FAILED"); return false; }
    Serial.printf("[Camera] Ready -- %dx%d fmt=%d len=%u\n",
                  fb->width, fb->height, fb->format, fb->len);
    esp_camera_fb_return(fb);
    return true;
}

// -- Saliency -> BlobResult adapter --------------------------------------------
// Converts SaliencyResult into BlobResult so behaviour layer is pipeline-agnostic.
// Uses a small persistent track table so blob IDs change when the saliency peak
// moves significantly -- this triggers proper dwell/shift in the behaviour layer.

static struct {
    float    normX = 0.5f, normY = 0.5f;
    uint8_t  id    = 1;
    uint32_t age   = 0;
    float    vx = 0, vy = 0;
    uint32_t lastMs = 0;
} _salTrack;

static BlobResult saliencyToBlobResult(const SaliencyResult& sal) {
    BlobResult res = {};
    if (!sal.valid) return res;

    uint32_t now = millis();
    float dt = (now - _salTrack.lastMs) / 1000.0f;
    if (dt <= 0 || dt > 2.0f) dt = 0.1f;

    // If peak has jumped significantly, treat as a new "blob" (new ID)
    float dx = sal.normX - _salTrack.normX;
    float dy = sal.normY - _salTrack.normY;
    float dist = sqrtf(dx*dx + dy*dy);
    if (dist > 0.15f) {
        _salTrack.id = (_salTrack.id % 254) + 1;  // new ID -> triggers gaze shift
        _salTrack.age = 0;
        Serial.printf("[Sal] Peak shift %.2f -> target %d\n", dist, _salTrack.id);
    }

    // Smooth track position
    _salTrack.normX = 0.7f * _salTrack.normX + 0.3f * sal.normX;
    _salTrack.normY = 0.7f * _salTrack.normY + 0.3f * sal.normY;
    _salTrack.vx    = 0.7f * _salTrack.vx + 0.3f * (dx / dt);
    _salTrack.vy    = 0.7f * _salTrack.vy + 0.3f * (dy / dt);
    _salTrack.age++;
    _salTrack.lastMs = now;

    res.anyMotion           = true;
    res.count               = 1;
    res.blobs[0].valid      = true;
    res.blobs[0].id         = _salTrack.id;
    res.blobs[0].normX      = _salTrack.normX;
    res.blobs[0].normY      = _salTrack.normY;
    res.blobs[0].normW      = 0.1f;
    res.blobs[0].normH      = 0.1f;
    res.blobs[0].area       = sal.confidence * 0.15f;
    res.blobs[0].score      = sal.confidence;
    res.blobs[0].age        = _salTrack.age;
    res.blobs[0].firstSeenMs = now;
    res.blobs[0].lastSeenMs  = now;
    res.blobs[0].vx         = _salTrack.vx;
    res.blobs[0].vy         = _salTrack.vy;
    return res;
}

// -- Saliency debug thumbnail ---------------------------------------------------
// Downsamples saliency map (SAL_WxSAL_H=80x60) -> DBG thumbnail (40x30)
static void saliencyToDebugThumb(uint8_t* motionThumb, uint8_t* blobThumb,
                                  const SaliencyResult& sal) {
    const uint8_t* salMap = saliency.getSaliencyMap();
    int bx = SAL_W / DBG_W, by = SAL_H / DBG_H;
    for (int dy = 0; dy < DBG_H; dy++)
        for (int dx = 0; dx < DBG_W; dx++) {
            uint32_t sum = 0;
            for (int yy=0; yy<by; yy++)
                for (int xx=0; xx<bx; xx++)
                    sum += salMap[(dy*by+yy)*SAL_W + (dx*bx+xx)];
            motionThumb[dy*DBG_W+dx] = (uint8_t)(sum/(bx*by));
        }

    memset(blobThumb, 0, DBG_W * DBG_H);
    if (sal.valid) {
        int tx = (int)(sal.normX * DBG_W);
        int ty = (int)(sal.normY * DBG_H);
        tx = constrain(tx, 0, DBG_W-1);
        ty = constrain(ty, 0, DBG_H-1);
        blobThumb[ty*DBG_W+tx] = 255;
        // Draw small cross
        if (tx>0) blobThumb[ty*DBG_W+tx-1] = 200;
        if (tx<DBG_W-1) blobThumb[ty*DBG_W+tx+1] = 200;
        if (ty>0) blobThumb[(ty-1)*DBG_W+tx] = 200;
        if (ty<DBG_H-1) blobThumb[(ty+1)*DBG_W+tx] = 200;
    }
}

// -- Core 0 vision task --------------------------------------------------------
static void visionTask(void* /*arg*/) {
    Serial.println("[Vision] Task running on Core 0");

    uint32_t frameCount = 0;
    uint32_t lastFpsMs  = millis();
    uint32_t fpsCount   = 0;
    float    fps        = 0;
    uint32_t failCount  = 0;

    // Also init saliency PSRAM now while we have time
    saliency.begin();

    // -- Background warmup: 5 seconds of aggressive learning ------------------
    // We collect real frames and learn WITHOUT motion gating for the first
    // N frames. This ensures the background model represents the actual scene
    // rather than a single snapshot. Motion gating re-enables after warmup.
    Serial.println("[Vision] Building background model (5s -- keep scene static)...");
    const int WARMUP_FRAMES = 55;  // ~5s at 11fps
    for (int w = 0; w < WARMUP_FRAMES; w++) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            // Downsample + frame average only -- no motion detection yet
            blobTracker.process(fb->buf, fb->width, fb->height);
            esp_camera_fb_return(fb);
        }
        if (w % 10 == 0)
            Serial.printf("[Vision] Warmup %d/%d\n", w, WARMUP_FRAMES);
        vTaskDelay(pdMS_TO_TICKS(90));
    }

    // Final reset: snapshot the averaged scene as background
    blobTracker.resetBackground();
    // Trigger fast warmup pass (30x learning passes on current frame)
    camera_fb_t* wfb = esp_camera_fb_get();
    if (wfb) {
        blobTracker.process(wfb->buf, wfb->width, wfb->height);
        esp_camera_fb_return(wfb);
    }

    // Allow 2.5s for the settle check to pass
    visionBgResetMs = millis();
    visionBgSettled = false;
    Serial.printf("[Vision] Background ready -- mode: %s\n",
                  activeVisionMode == VisionMode::BLOBS ? "BLOBS" : "SALIENCY");

    for (;;) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            if (++failCount % 20 == 1)
                Serial.printf("[Vision] Frame grab failed (%lu)\n", (unsigned long)failCount);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        failCount = 0;
        frameCount++;
        fpsCount++;

        // -- Scheduled background reset check -----------------------------------
        if (_scheduledResetMs > 0 && millis() >= _scheduledResetMs) {
            _scheduledResetMs = 0;
            blobTracker.resetBackground();
            visionBgResetMs  = millis();
            visionBgSettled  = false;
            Serial.println("[Vision] Scheduled background reset executed");
        }

        // -- Background settle check ------------------------------------------
        // Mark settled when: enough time has passed AND the blob count is
        // low (< 2 blobs). If the scene is still chaotic after a reset,
        // keep waiting -- don't allow tracking until things calm down.
        BlobResult result;

        // Background settle: just wait 4s after last reset.
        if (!visionBgSettled && visionBgResetMs > 0 &&
            (millis() - visionBgResetMs) > 4000) {
            visionBgSettled = true;
            Serial.printf("[Vision] Background settled (%d blobs)\n",
                          result.count);
        }
        VisionMode mode = activeVisionMode;  // snapshot -- avoids race

        if (mode == VisionMode::BLOBS) {
            // -- Blob tracker pipeline -----------------------------------------
            result = blobTracker.process(fb->buf, fb->width, fb->height);

        } else {
            // -- Saliency pipeline ---------------------------------------------
            SaliencyResult sal = saliency.process(fb->buf, fb->width, fb->height);
            result = saliencyToBlobResult(sal);
        }

        esp_camera_fb_return(fb);

        // Post result (overwrite -- always freshest)
        xQueueOverwrite(blobQueue, &result);

        // Update debug thumbnails
        if (xSemaphoreTake(debugMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            if (mode == VisionMode::BLOBS) {
                memcpy(visionDebug.motion, blobTracker.getMotionThumb(), DBG_W*DBG_H);
                memcpy(visionDebug.blobs,  blobTracker.getBlobThumb(),   DBG_W*DBG_H);
            } else {
                SaliencyResult sal = {};  // reuse last -- saliency caches internally
                // Re-fetch from saliency map for debug
                sal.valid = result.count > 0;
                if (sal.valid) {
                    sal.normX = result.blobs[0].normX;
                    sal.normY = result.blobs[0].normY;
                    sal.confidence = result.blobs[0].score;
                }
                saliencyToDebugThumb(visionDebug.motion, visionDebug.blobs, sal);
            }
            visionDebug.blobCount  = result.count;
            visionDebug.frameCount = frameCount;
            visionDebug.fps        = fps;
            visionDebug.mode       = mode;
            xSemaphoreGive(debugMutex);
        }

        // FPS
        uint32_t now = millis();
        if (now - lastFpsMs >= 2000) {
            fps = fpsCount * 1000.0f / (now - lastFpsMs);
            fpsCount = 0; lastFpsMs = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void visionTaskStart() {
    blobQueue  = xQueueCreate(1, sizeof(BlobResult));
    debugMutex = xSemaphoreCreateMutex();
    configASSERT(blobQueue);
    configASSERT(debugMutex);
    xTaskCreatePinnedToCore(visionTask, "vision", 10240, nullptr, 5, nullptr, 0);
}

bool visionGetDebug(VisionDebug& out) {
    if (!debugMutex) return false;
    if (xSemaphoreTake(debugMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    memcpy(&out, &visionDebug, sizeof(VisionDebug));
    xSemaphoreGive(debugMutex);
    return true;
}