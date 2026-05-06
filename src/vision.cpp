#include "vision.h"
#include "saliency.h"
#include <esp_camera.h>

// ── Shared state ──────────────────────────────────────────────────────────────
QueueHandle_t      detectionQueue = nullptr;
SemaphoreHandle_t  debugMutex     = nullptr;
VisionDebug        visionDebug    = {};

// ── Camera init ───────────────────────────────────────────────────────────────
// Strategy: init in JPEG at UXGA to correctly size DMA buffers (OV3660 requirement),
// verify grab works, then reinit in GRAYSCALE at QVGA for saliency processing.
// Grayscale gives us raw pixel bytes directly — no JPEG decoding needed.
bool cameraInit() {
    // ── Step 1: JPEG UXGA init to size DMA buffers ────────────────────────────
    auto doInit = [](pixformat_t fmt, framesize_t sz) -> bool {
        camera_config_t cfg = {};
        cfg.ledc_channel = LEDC_CHANNEL_0; cfg.ledc_timer = LEDC_TIMER_0;
        cfg.pin_d0 = CAM_PIN_D0; cfg.pin_d1 = CAM_PIN_D1;
        cfg.pin_d2 = CAM_PIN_D2; cfg.pin_d3 = CAM_PIN_D3;
        cfg.pin_d4 = CAM_PIN_D4; cfg.pin_d5 = CAM_PIN_D5;
        cfg.pin_d6 = CAM_PIN_D6; cfg.pin_d7 = CAM_PIN_D7;
        cfg.pin_xclk     = CAM_PIN_XCLK;  cfg.pin_pclk  = CAM_PIN_PCLK;
        cfg.pin_vsync    = CAM_PIN_VSYNC;  cfg.pin_href  = CAM_PIN_HREF;
        cfg.pin_sccb_sda = CAM_PIN_SIOD;   cfg.pin_sccb_scl = CAM_PIN_SIOC;
        cfg.pin_pwdn     = CAM_PIN_PWDN;   cfg.pin_reset = CAM_PIN_RESET;
        cfg.xclk_freq_hz = CAM_XCLK_FREQ;
        cfg.pixel_format = fmt;
        cfg.frame_size   = sz;
        cfg.jpeg_quality = 10;
        cfg.fb_location  = CAMERA_FB_IN_PSRAM;
        // GRAYSCALE needs fb_count=2 on S3 for continuous DMA
        cfg.fb_count     = (fmt == PIXFORMAT_GRAYSCALE) ? 2 : (psramFound() ? 2 : 1);
        cfg.grab_mode    = CAMERA_GRAB_LATEST;
        return esp_camera_init(&cfg) == ESP_OK;
    };

    // Phase 1: JPEG UXGA — sizes DMA correctly
    Serial.println("[Camera] Phase 1: JPEG UXGA init...");
    if (!doInit(PIXFORMAT_JPEG, FRAMESIZE_UXGA)) {
        Serial.println("[Camera] JPEG UXGA init FAILED");
        return false;
    }
    // Quick verify
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[Camera] JPEG grab FAILED"); return false; }
    Serial.printf("[Camera] JPEG grab OK — %dx%d len=%u\n", fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);

    // Phase 2: reinit as GRAYSCALE QVGA for saliency
    Serial.println("[Camera] Phase 2: reinit GRAYSCALE QVGA...");
    esp_camera_deinit();
    delay(100);
    if (!doInit(PIXFORMAT_GRAYSCALE, FRAMESIZE_QVGA)) {
        Serial.println("[Camera] GRAYSCALE init FAILED — falling back to JPEG");
        // Fall back to JPEG QVGA — extractGrey will use Y-approx
        if (!doInit(PIXFORMAT_JPEG, FRAMESIZE_QVGA)) return false;
    }

    // Apply sensor tweaks
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        Serial.printf("[Camera] Sensor: 0x%04X (%s)\n", s->id.PID,
                      s->id.PID == 0x3660 ? "OV3660" :
                      s->id.PID == 0x26   ? "OV2640" :
                      s->id.PID == 0x5640 ? "OV5640" : "?");
        if (s->id.PID == 0x3660) {
            s->set_vflip(s, 1);
            s->set_brightness(s, 1);
            s->set_saturation(s, -2);
        }
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gainceiling(s, (gainceiling_t)4);
        s->set_lenc(s, 1);
    }
    delay(300); // let AEC settle

    // Final verify
    fb = esp_camera_fb_get();
    if (!fb) { Serial.println("[Camera] Final grab FAILED"); return false; }
    Serial.printf("[Camera] Ready — %dx%d fmt=%d len=%u (expect %d for GREY)\n",
                  fb->width, fb->height, fb->format, fb->len,
                  fb->width * fb->height);
    esp_camera_fb_return(fb);
    return true;
}


// ── Convert JPEG fb to greyscale (simple: use Y channel approximation) ────────
// For JPEG format we re-init to grayscale mode after confirming camera works
bool switchToGrayscale() {
    // Deinit and reinit in grayscale mode
    esp_camera_deinit();
    delay(100);

    camera_config_t cfg = {};
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = CAM_PIN_D0;  cfg.pin_d1  = CAM_PIN_D1;
    cfg.pin_d2       = CAM_PIN_D2;  cfg.pin_d3  = CAM_PIN_D3;
    cfg.pin_d4       = CAM_PIN_D4;  cfg.pin_d5  = CAM_PIN_D5;
    cfg.pin_d6       = CAM_PIN_D6;  cfg.pin_d7  = CAM_PIN_D7;
    cfg.pin_xclk     = CAM_PIN_XCLK; cfg.pin_pclk  = CAM_PIN_PCLK;
    cfg.pin_vsync    = CAM_PIN_VSYNC; cfg.pin_href  = CAM_PIN_HREF;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;  cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_pwdn     = CAM_PIN_PWDN;  cfg.pin_reset = CAM_PIN_RESET;
    cfg.xclk_freq_hz = CAM_XCLK_FREQ;
    cfg.sccb_i2c_port = 1;           // keep on peripheral 1
    cfg.pixel_format = PIXFORMAT_GRAYSCALE;
    cfg.frame_size   = CAM_FRAME_SIZE;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[Camera] Grayscale reinit FAILED: 0x%x\n", err);
        Serial.println("[Camera] Falling back to JPEG mode — saliency on Y channel");
        return false;
    }

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[Camera] Grayscale test grab failed — reverting");
        return false;
    }
    Serial.printf("[Camera] Grayscale OK — %dx%d len=%u\n",
                  fb->width, fb->height, fb->len);
    esp_camera_fb_return(fb);
    return true;
}

// ── Extract greyscale from framebuffer ────────────────────────────────────────
static void extractGrey(camera_fb_t* fb, uint8_t* outBuf, int w, int h) {
    if (fb->format == PIXFORMAT_GRAYSCALE) {
        // Direct copy — this is the normal path after grayscale reinit
        memcpy(outBuf, fb->buf, w * h);
    } else if (fb->format == PIXFORMAT_RGB565) {
        // R*0.299 + G*0.587 + B*0.114 approximation
        for (int i = 0; i < w*h; i++) {
            uint16_t px = ((uint16_t)fb->buf[i*2] << 8) | fb->buf[i*2+1];
            uint8_t r = (px >> 11) & 0x1F;
            uint8_t g = (px >> 5)  & 0x3F;
            uint8_t b =  px        & 0x1F;
            outBuf[i] = (uint8_t)((r * 77 + g * 38 + b * 15) >> 5);
        }
    } else {
        // JPEG — we shouldn't be here after reinit, but handle gracefully
        // Signal in serial so we can detect the fallback being used
        static uint32_t warnCount = 0;
        if (++warnCount % 100 == 1)
            Serial.printf("[Vision] WARN: JPEG format in extractGrey — reinit failed\n");
        memset(outBuf, 128, w * h);
    }
}

// ── Downsampling helpers ──────────────────────────────────────────────────────
static void downsampleToDebug(const uint8_t* src, int sw, int sh,
                               uint8_t* dst, int dw, int dh) {
    int bx = sw / dw, by = sh / dh;
    for (int dy = 0; dy < dh; dy++)
        for (int dx = 0; dx < dw; dx++) {
            uint32_t sum = 0;
            for (int yy = 0; yy < by; yy++)
                for (int xx = 0; xx < bx; xx++)
                    sum += src[(dy*by+yy)*sw + (dx*bx+xx)];
            dst[dy*dw+dx] = sum / (bx*by);
        }
}

static void downsampleSalToDebug(const uint8_t* src, int sw, int sh,
                                  uint8_t* dst, int dw, int dh) {
    int bx = sw/dw, by = sh/dh;
    for (int dy=0; dy<dh; dy++)
        for (int dx=0; dx<dw; dx++) {
            uint32_t sum=0;
            for (int yy=0; yy<by; yy++)
                for (int xx=0; xx<bx; xx++)
                    sum += src[(dy*by+yy)*sw+(dx*bx+xx)];
            dst[dy*dw+dx] = sum/(bx*by);
        }
}

// ── Greyscale working buffer (PSRAM) ──────────────────────────────────────────
static uint8_t* greyBuf = nullptr;

// ── Core 0 vision task ────────────────────────────────────────────────────────
static void visionTask(void* /*arg*/) {
    Serial.println("[Vision] Task running on Core 0");

    // Allocate greyscale buffer in PSRAM
    greyBuf = (uint8_t*)ps_malloc(320 * 240);
    if (!greyBuf) {
        Serial.println("[Vision] PSRAM alloc failed for grey buffer!");
        vTaskDelete(nullptr);
        return;
    }

    uint32_t frameCount = 0;
    uint32_t lastFpsMs  = millis();
    uint32_t fpsCount   = 0;
    uint32_t fps        = 0;
    uint32_t failCount  = 0;

    for (;;) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            failCount++;
            if (failCount % 10 == 1) {
                Serial.printf("[Vision] Frame grab failed (%lu total)\n", failCount);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        failCount = 0;  // reset on success

        frameCount++;
        fpsCount++;

        // Extract greyscale
        extractGrey(fb, greyBuf, fb->width, fb->height);
        esp_camera_fb_return(fb);

        // Run saliency on greyscale
        SaliencyResult sal = saliency.process(greyBuf, 320, 240);

        // Post to queue (overwrite stale)
        DetectionResult det;
        det.valid      = sal.valid;
        det.normX      = sal.normX;
        det.normY      = sal.normY;
        det.confidence = sal.confidence;
        det.timestamp  = millis();
        xQueueOverwrite(detectionQueue, &det);

        // Update debug thumbnails
        if (xSemaphoreTake(debugMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            downsampleToDebug(greyBuf, 320, 240,
                              visionDebug.grey, DBG_W, DBG_H);
            downsampleSalToDebug(saliency.getSaliencyMap(), SAL_W, SAL_H,
                                 visionDebug.saliency, DBG_W, DBG_H);
            downsampleSalToDebug(saliency.getMotionMap(), SAL_W, SAL_H,
                                 visionDebug.motion, DBG_W, DBG_H);
            visionDebug.targetX    = sal.valid ? (int)(sal.normX * DBG_W) : -1;
            visionDebug.targetY    = sal.valid ? (int)(sal.normY * DBG_H) : -1;
            visionDebug.targetConf = sal.confidence;
            visionDebug.frameCount = frameCount;
            visionDebug.fps        = fps;
            xSemaphoreGive(debugMutex);
        }

        // FPS counter
        uint32_t now = millis();
        if (now - lastFpsMs >= 2000) {
            fps       = fpsCount * 1000 / (now - lastFpsMs);
            fpsCount  = 0;
            lastFpsMs = now;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── visionTaskStart ───────────────────────────────────────────────────────────
void visionTaskStart() {
    detectionQueue = xQueueCreate(1, sizeof(DetectionResult));
    configASSERT(detectionQueue);
    debugMutex = xSemaphoreCreateMutex();
    configASSERT(debugMutex);

    xTaskCreatePinnedToCore(
        visionTask, "vision",
        8192, nullptr,
        5, nullptr,
        0   // Core 0
    );
}

// ── visionGetDebug ────────────────────────────────────────────────────────────
bool visionGetDebug(VisionDebug& out) {
    if (!debugMutex) return false;
    if (xSemaphoreTake(debugMutex, pdMS_TO_TICKS(10)) != pdTRUE) return false;
    memcpy(&out, &visionDebug, sizeof(VisionDebug));
    xSemaphoreGive(debugMutex);
    return true;
}