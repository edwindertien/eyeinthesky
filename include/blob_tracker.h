#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════════════
//  blob_tracker.h — pure motion blob detection and multi-target tracking
//
//  Pipeline (replaces saliency.h/saliency.cpp entirely):
//    1. Downsample QVGA → 80×60 working resolution
//    2. Background subtraction (EMA) → motion mask
//    3. Morphological open (erode+dilate) to remove noise specks
//    4. Connected component labelling → raw blobs
//    5. Filter: min/max size, merge nearby blobs
//    6. Track blobs across frames by nearest-neighbour matching
//    7. Score blobs by size + motion magnitude + track age
//    8. Return ordered list of up to MAX_BLOBS active blobs
//
//  Blob identity:
//    Each blob gets an ID that persists across frames as long as it
//    stays near the same position. This lets the behaviour layer
//    say "look at blob 2" and have that mean the same person for
//    multiple seconds even if they pause briefly.
//
//  Background reset:
//    resetBackground() forces a full re-learn. Called on wake from sleep
//    and periodically (every BG_RESET_INTERVAL_MS) to handle lighting changes.
// ═══════════════════════════════════════════════════════════════════════════

// Working resolution — QVGA (320×240) / 4
#define BT_W    80
#define BT_H    60
#define BT_PIX  (BT_W * BT_H)

// Max blobs tracked simultaneously
#define MAX_BLOBS   5

// Debug thumbnail resolution (same as before for ANSI display)
#define DBG_W   40
#define DBG_H   30

struct Blob {
    bool     valid;          // Is this slot active?
    uint8_t  id;             // Persistent ID (1–255, wraps)
    float    normX, normY;   // Centre position (0–1)
    float    normW, normH;   // Bounding box size (0–1)
    float    area;           // Fraction of frame covered (0–1)
    float    score;          // Combined quality score (0–1)
    uint32_t firstSeenMs;    // When this blob was first detected
    uint32_t lastSeenMs;     // Last frame it was matched
    uint32_t age;            // Frames since first detected
    float    vx, vy;         // Velocity (normalised, pixels/frame)
};

struct BlobResult {
    int      count;              // Number of valid blobs (0 = nothing moving)
    Blob     blobs[MAX_BLOBS];   // Sorted by score (highest first)
    bool     anyMotion;          // True if anything above noise floor
};

class BlobTracker {
public:
    bool begin();

    // Process a QVGA greyscale frame — call every frame
    BlobResult process(const uint8_t* grey, int w, int h);

    // Force full background reset (on wake, or periodically)
    void resetBackground();

    // ── Debug maps for ANSI display ──────────────────────────────────────────
    // Motion mask (0=still, 255=moving) at DBG resolution
    const uint8_t* getMotionThumb()  const { return _motionThumb; }
    // Blob map (0=no blob, blobId=blob) at DBG resolution
    const uint8_t* getBlobThumb()    const { return _blobThumb; }

    // ── Tunable parameters (serial commands) ─────────────────────────────────
    int     bgAlphaInt      = 15;     // background learning rate ×1000
                                      // 15→converges in ~65 frames (~6s at 11fps)
    int     motionThreshold = 25;     // pixel diff to count as motion (0–255)
    int     minBlobPixels   = 15;     // min blob size in SAL pixels (~0.3% frame)
    int     maxBlobPixels   = 2000;   // max blob size (ignore huge regions)
    int     mergeRadius     = 8;      // merge blobs closer than this (SAL pixels)
    float   matchRadius     = 0.15f;  // max normalised distance for blob tracking
    uint32_t blobTimeoutMs  = 1500;   // remove blob if not seen for this long
    uint32_t bgResetIntervalMs = 300000; // periodic bg reset (5 min)
    int     minBlobAge      = 6;      // raised — more conservative blob acceptance      // frames a blob must exist before reporting
                                      // (filters lamp flicker — short-lived blobs)
    int     frameAvgCount   = 3;      // frames to average before background compare
                                      // (cancels 50Hz lamp flicker at ~11fps)
    int     maxPositionVar  = 6;      // max position variance (SAL px) for static
                                      // blob rejection — lamp stays fixed, person moves
    float   shiftThreshold  = 0.60f;  // fraction of frame with motion → camera shift
                                      // person=<0.20, camera shift=>0.60

private:
    // Working buffers in PSRAM
    uint8_t* _bgModel    = nullptr;   // background EMA (BT_W × BT_H)
    uint8_t* _motionMask = nullptr;   // thresholded motion (BT_W × BT_H)
    uint8_t* _morphBuf   = nullptr;   // morphology scratch (BT_W × BT_H)
    uint8_t* _labelBuf   = nullptr;   // connected component labels (BT_W × BT_H)
    uint8_t* _workGrey   = nullptr;   // downsampled frame (BT_W × BT_H)
    // Frame averaging: ring buffer of last N frames for flicker cancellation
    static const int MAX_AVG = 4;
    uint8_t* _avgBuf[MAX_AVG] = {};   // ring buffer frames
    uint8_t* _avgGrey = nullptr;      // averaged output
    int      _avgHead = 0;            // ring buffer write head
    int      _avgFilled = 0;          // how many frames in buffer so far

    // Debug thumbnails (DBG_W × DBG_H)
    uint8_t  _motionThumb[DBG_W * DBG_H] = {};
    uint8_t  _blobThumb[DBG_W * DBG_H]   = {};

    // Persistent blob tracks
    Blob     _tracks[MAX_BLOBS]  = {};
    uint8_t  _nextId             = 1;
    bool     _bgReady            = false;
    uint32_t _lastBgReset        = 0;
    uint32_t _lastFrameMs        = 0;

    // Processing steps
    void _downsample(const uint8_t* src, int sw, int sh);
    void _updateFrameAverage();
    void _updateBackground();
    void _computeMotion();
    void _morphOpen(int radius);       // erode then dilate (removes noise)
    int  _labelBlobs(Blob* raw, int maxRaw);  // returns blob count
    void _matchAndUpdate(Blob* raw, int rawCount, uint32_t now);
    void _updateDebugThumbs(uint32_t now);
    BlobResult _buildResult(uint32_t now);
};

extern BlobTracker blobTracker;