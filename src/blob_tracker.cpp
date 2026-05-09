#include "blob_tracker.h"
#include <cstring>
#include <cmath>

BlobTracker blobTracker;

// -- begin ---------------------------------------------------------------------
bool BlobTracker::begin() {
    _bgModel    = (uint8_t*)ps_malloc(BT_PIX);
    _motionMask = (uint8_t*)ps_malloc(BT_PIX);
    _morphBuf   = (uint8_t*)ps_malloc(BT_PIX);
    _labelBuf   = (uint8_t*)ps_malloc(BT_PIX);
    _workGrey   = (uint8_t*)ps_malloc(BT_PIX);

    // Frame averaging buffers
    _avgGrey = (uint8_t*)ps_malloc(BT_PIX);
    for (int i = 0; i < MAX_AVG; i++)
        _avgBuf[i] = (uint8_t*)ps_malloc(BT_PIX);

    if (!_bgModel || !_motionMask || !_morphBuf || !_labelBuf ||
        !_workGrey || !_avgGrey) {
        Serial.println("[BlobTracker] PSRAM alloc FAILED");
        return false;
    }
    for (int i = 0; i < MAX_AVG; i++) {
        if (!_avgBuf[i]) {
            Serial.println("[BlobTracker] PSRAM alloc FAILED (avgBuf)");
            return false;
        }
        memset(_avgBuf[i], 128, BT_PIX);
    }
    memset(_avgGrey, 128, BT_PIX);
    _avgHead = 0; _avgFilled = 0;

    memset(_bgModel,    128, BT_PIX);
    memset(_motionMask,   0, BT_PIX);
    memset(_morphBuf,     0, BT_PIX);
    memset(_labelBuf,     0, BT_PIX);
    memset(_workGrey,   128, BT_PIX);
    memset(_motionThumb,  0, sizeof(_motionThumb));
    memset(_blobThumb,    0, sizeof(_blobThumb));

    for (int i = 0; i < MAX_BLOBS; i++) _tracks[i] = {};

    Serial.printf("[BlobTracker] Ready -- %u bytes PSRAM, working res %dx%d\n",
                  (unsigned)(BT_PIX * 5), BT_W, BT_H);
    return true;
}

// -- resetBackground -----------------------------------------------------------
void BlobTracker::resetBackground() {
    // Snapshot current averaged frame as background starting point
    if (_avgGrey && _avgFilled > 0)
        memcpy(_bgModel, _avgGrey, BT_PIX);
    else if (_workGrey)
        memcpy(_bgModel, _workGrey, BT_PIX);
    else
        memset(_bgModel, 128, BT_PIX);

    // Clear frame average buffer
    _avgHead = 0; _avgFilled = 0;
    if (_avgGrey) memset(_avgGrey, 128, BT_PIX);
    for (int i = 0; i < MAX_AVG; i++)
        if (_avgBuf[i]) memset(_avgBuf[i], 128, BT_PIX);

    // Clear all tracks -- fresh start
    for (int t = 0; t < MAX_BLOBS; t++) _tracks[t] = {};

    // Set _bgReady=false so process() runs the fast warmup on next call
    _bgReady     = false;
    _lastBgReset = millis();
    Serial.println("[BlobTracker] Background reset -- will re-learn on next frame");
}

// -- _downsample ---------------------------------------------------------------
void BlobTracker::_downsample(const uint8_t* src, int sw, int sh) {
    // QVGA (320x240) -> BT (80x60): factor 4x4
    int bx = sw / BT_W, by = sh / BT_H;
    for (int dy = 0; dy < BT_H; dy++)
        for (int dx = 0; dx < BT_W; dx++) {
            uint32_t sum = 0;
            for (int yy = 0; yy < by; yy++)
                for (int xx = 0; xx < bx; xx++)
                    sum += src[(dy*by+yy)*sw + (dx*bx+xx)];
            _workGrey[dy*BT_W+dx] = (uint8_t)(sum / (bx*by));
        }
}

// -- _updateFrameAverage -------------------------------------------------------
// Rolling average of last N downsampled frames.
// Cancels 50Hz lamp flicker which alternates between frames.
// _avgGrey contains the averaged result used for motion detection.
void BlobTracker::_updateFrameAverage() {
    // Write new frame into ring buffer
    memcpy(_avgBuf[_avgHead], _workGrey, BT_PIX);
    _avgHead = (_avgHead + 1) % MAX_AVG;
    if (_avgFilled < MAX_AVG) _avgFilled++;

    int n = max(1, min(_avgFilled, frameAvgCount));

    // Average the last n frames
    for (int i = 0; i < BT_PIX; i++) {
        uint32_t sum = 0;
        for (int f = 0; f < n; f++) {
            int idx = (_avgHead - 1 - f + MAX_AVG) % MAX_AVG;
            sum += _avgBuf[idx][i];
        }
        _avgGrey[i] = (uint8_t)(sum / n);
    }
}

// -- _updateBackground ---------------------------------------------------------
// Motion-gated EMA -- only update background at still pixels
void BlobTracker::_updateBackground() {
    int alpha = bgAlphaInt;
    for (int i = 0; i < BT_PIX; i++) {
        if (_motionMask[i]) continue;   // don't absorb moving pixels
        int bg = _bgModel[i];
        // Use averaged frame for background learning too
        _bgModel[i] = (uint8_t)(bg + (alpha * ((int)_avgGrey[i] - bg)) / 1000);
    }
}

// -- _computeMotion ------------------------------------------------------------
void BlobTracker::_computeMotion() {
    // Compare averaged frame to background -- flicker is averaged away
    int thr = motionThreshold;
    for (int i = 0; i < BT_PIX; i++) {
        int diff = abs((int)_avgGrey[i] - (int)_bgModel[i]);
        _motionMask[i] = (diff >= thr) ? (uint8_t)min(diff, 255) : 0;
    }
}

// -- _morphOpen ----------------------------------------------------------------
// Erosion then dilation with a box of given radius.
// Removes isolated noise pixels while preserving larger blobs.
void BlobTracker::_morphOpen(int radius) {
    // Erode into _morphBuf
    for (int y = 0; y < BT_H; y++) {
        for (int x = 0; x < BT_W; x++) {
            uint8_t minV = 255;
            for (int dy = -radius; dy <= radius; dy++) {
                int ny = y+dy; if (ny<0||ny>=BT_H) { minV=0; break; }
                for (int dx = -radius; dx <= radius; dx++) {
                    int nx = x+dx; if (nx<0||nx>=BT_W) { minV=0; break; }
                    if (_motionMask[ny*BT_W+nx] < minV)
                        minV = _motionMask[ny*BT_W+nx];
                }
            }
            _morphBuf[y*BT_W+x] = minV;
        }
    }
    // Dilate back into _motionMask
    for (int y = 0; y < BT_H; y++) {
        for (int x = 0; x < BT_W; x++) {
            uint8_t maxV = 0;
            for (int dy = -radius; dy <= radius; dy++) {
                int ny = y+dy; if (ny<0||ny>=BT_H) continue;
                for (int dx = -radius; dx <= radius; dx++) {
                    int nx = x+dx; if (nx<0||nx>=BT_W) continue;
                    if (_morphBuf[ny*BT_W+nx] > maxV)
                        maxV = _morphBuf[ny*BT_W+nx];
                }
            }
            _motionMask[y*BT_W+x] = maxV;
        }
    }
}

// -- _labelBlobs ---------------------------------------------------------------
// Simple connected component labelling via flood fill.
// Returns number of blobs found, fills raw[] with blob data.
int BlobTracker::_labelBlobs(Blob* raw, int maxRaw) {
    memset(_labelBuf, 0, BT_PIX);
    int blobCount = 0;
    uint8_t nextLabel = 1;

    // Stack-based flood fill to avoid recursion overflow
    static int16_t stack[BT_PIX * 2];

    for (int y = 0; y < BT_H && blobCount < maxRaw; y++) {
        for (int x = 0; x < BT_W && blobCount < maxRaw; x++) {
            if (_motionMask[y*BT_W+x] == 0) continue;
            if (_labelBuf[y*BT_W+x] != 0)   continue;

            // Flood fill this blob
            uint8_t label = nextLabel++;
            int top = 0;
            stack[top++] = (int16_t)x;
            stack[top++] = (int16_t)y;

            int   pixCount = 0;
            int   sumX = 0, sumY = 0;
            int   minX = x, maxX = x, minY = y, maxY = y;
            float sumMag = 0;

            while (top > 0) {
                int16_t cy = stack[--top];
                int16_t cx = stack[--top];
                int idx = cy*BT_W+cx;
                if (cx<0||cx>=BT_W||cy<0||cy>=BT_H) continue;
                if (_motionMask[idx] == 0)  continue;
                if (_labelBuf[idx] != 0)    continue;

                _labelBuf[idx] = label;
                pixCount++;
                sumX += cx; sumY += cy;
                sumMag += _motionMask[idx] / 255.0f;
                if (cx < minX) minX = cx; if (cx > maxX) maxX = cx;
                if (cy < minY) minY = cy; if (cy > maxY) maxY = cy;

                // Push 4-connected neighbours
                if (top < (int)(sizeof(stack)/sizeof(stack[0])) - 4) {
                    stack[top++] = cx+1; stack[top++] = cy;
                    stack[top++] = cx-1; stack[top++] = cy;
                    stack[top++] = cx;   stack[top++] = cy+1;
                    stack[top++] = cx;   stack[top++] = cy-1;
                }
            }

            // Filter by size
            if (pixCount < minBlobPixels || pixCount > maxBlobPixels) {
                // Too small or too large -- unlabel so it's invisible
                continue;
            }

            Blob& b = raw[blobCount++];
            b.valid    = true;
            b.id       = 0;  // assigned during matching
            b.normX    = (sumX / (float)pixCount) / BT_W;
            b.normY    = (sumY / (float)pixCount) / BT_H;
            b.normW    = (float)(maxX - minX + 1) / BT_W;
            b.normH    = (float)(maxY - minY + 1) / BT_H;
            b.area     = (float)pixCount / BT_PIX;
            b.score    = sumMag / pixCount;  // mean motion magnitude
            b.vx = b.vy = 0;
        }
    }

    // Sort by area descending (largest blob first)
    for (int i = 0; i < blobCount-1; i++)
        for (int j = i+1; j < blobCount; j++)
            if (raw[j].area > raw[i].area) {
                Blob tmp = raw[i]; raw[i] = raw[j]; raw[j] = tmp;
            }

    return blobCount;
}

// -- _matchAndUpdate -----------------------------------------------------------
// Match raw blobs to existing tracks by nearest-neighbour.
// Updates track positions, creates new tracks, expires stale ones.
void BlobTracker::_matchAndUpdate(Blob* raw, int rawCount, uint32_t now) {
    bool matched[MAX_BLOBS] = {};      // which raw blobs got matched
    float dt = (now - _lastFrameMs) / 1000.0f;  // seconds since last frame
    if (dt <= 0.0f || dt > 1.0f) dt = 0.1f;

    // Match existing tracks to raw blobs
    for (int t = 0; t < MAX_BLOBS; t++) {
        if (!_tracks[t].valid) continue;

        float bestDist = matchRadius;
        int   bestRaw  = -1;
        for (int r = 0; r < rawCount; r++) {
            if (matched[r]) continue;
            float dx = raw[r].normX - _tracks[t].normX;
            float dy = raw[r].normY - _tracks[t].normY;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < bestDist) { bestDist = d; bestRaw = r; }
        }

        if (bestRaw >= 0) {
            // Update track
            matched[bestRaw] = true;
            Blob& r = raw[bestRaw];
            // Velocity (smoothed)
            float newVx = (r.normX - _tracks[t].normX) / dt;
            float newVy = (r.normY - _tracks[t].normY) / dt;
            _tracks[t].vx = 0.7f * _tracks[t].vx + 0.3f * newVx;
            _tracks[t].vy = 0.7f * _tracks[t].vy + 0.3f * newVy;
            // Smooth position
            _tracks[t].normX = 0.6f * _tracks[t].normX + 0.4f * r.normX;
            _tracks[t].normY = 0.6f * _tracks[t].normY + 0.4f * r.normY;
            _tracks[t].normW = r.normW;
            _tracks[t].normH = r.normH;
            _tracks[t].area  = r.area;
            _tracks[t].score = r.score;
            _tracks[t].lastSeenMs = now;
            _tracks[t].age++;
        } else {
            // No match -- expire if too old
            if (now - _tracks[t].lastSeenMs > blobTimeoutMs) {
                _tracks[t].valid = false;
                Serial.printf("[Blob] Track %d expired\n", _tracks[t].id);
            }
        }
    }

    // Create new tracks for unmatched raw blobs
    for (int r = 0; r < rawCount; r++) {
        if (matched[r]) continue;
        // Find empty track slot
        for (int t = 0; t < MAX_BLOBS; t++) {
            if (_tracks[t].valid) continue;
            _tracks[t]            = raw[r];
            _tracks[t].id         = _nextId++;
            if (_nextId == 0) _nextId = 1;
            _tracks[t].firstSeenMs = now;
            _tracks[t].lastSeenMs  = now;
            _tracks[t].age         = 1;
            _tracks[t].vx = _tracks[t].vy = 0;
            Serial.printf("[Blob] New track %d at %.2f,%.2f area=%.3f\n",
                          _tracks[t].id, _tracks[t].normX, _tracks[t].normY,
                          _tracks[t].area);
            break;
        }
    }
}

// -- _updateDebugThumbs --------------------------------------------------------
void BlobTracker::_updateDebugThumbs(uint32_t now) {
    // Motion thumb: downsample _motionMask BT->DBG (factor 2)
    int bx = BT_W / DBG_W, by = BT_H / DBG_H;
    for (int dy = 0; dy < DBG_H; dy++)
        for (int dx = 0; dx < DBG_W; dx++) {
            uint32_t sum = 0;
            for (int yy=0; yy<by; yy++)
                for (int xx=0; xx<bx; xx++)
                    sum += _motionMask[(dy*by+yy)*BT_W + (dx*bx+xx)];
            _motionThumb[dy*DBG_W+dx] = (uint8_t)(sum/(bx*by));
        }

    // Blob thumb: draw blob bounding boxes
    memset(_blobThumb, 0, sizeof(_blobThumb));
    for (int t = 0; t < MAX_BLOBS; t++) {
        if (!_tracks[t].valid) continue;
        if (now - _tracks[t].lastSeenMs > 200) continue;
        // Draw blob centroid marker
        int cx = (int)(_tracks[t].normX * DBG_W);
        int cy = (int)(_tracks[t].normY * DBG_H);
        cx = constrain(cx, 0, DBG_W-1);
        cy = constrain(cy, 0, DBG_H-1);
        // Fill bounding box
        int x0 = (int)((_tracks[t].normX - _tracks[t].normW/2) * DBG_W);
        int x1 = (int)((_tracks[t].normX + _tracks[t].normW/2) * DBG_W);
        int y0 = (int)((_tracks[t].normY - _tracks[t].normH/2) * DBG_H);
        int y1 = (int)((_tracks[t].normY + _tracks[t].normH/2) * DBG_H);
        x0=constrain(x0,0,DBG_W-1); x1=constrain(x1,0,DBG_W-1);
        y0=constrain(y0,0,DBG_H-1); y1=constrain(y1,0,DBG_H-1);
        // Draw border
        for (int x=x0; x<=x1; x++) {
            _blobThumb[y0*DBG_W+x] = _tracks[t].id * 40 + 100;
            _blobThumb[y1*DBG_W+x] = _tracks[t].id * 40 + 100;
        }
        for (int y=y0; y<=y1; y++) {
            _blobThumb[y*DBG_W+x0] = _tracks[t].id * 40 + 100;
            _blobThumb[y*DBG_W+x1] = _tracks[t].id * 40 + 100;
        }
        // Centre mark
        _blobThumb[cy*DBG_W+cx] = 255;
    }
}

// -- _buildResult --------------------------------------------------------------
BlobResult BlobTracker::_buildResult(uint32_t now) {
    BlobResult res = {};

    // Score tracks: area x age_bonus x recency
    // Older tracks (more confident) score higher.
    // Blobs younger than minBlobAge are suppressed (filters lamp flicker).
    for (int t = 0; t < MAX_BLOBS; t++) {
        if (!_tracks[t].valid) continue;
        if (now - _tracks[t].lastSeenMs > blobTimeoutMs) continue;

        // Age filter: ignore new blobs until they've persisted N frames
        // A lamp flicker creates a blob for 1-2 frames; a person for many
        if ((int)_tracks[t].age < minBlobAge) continue;

        // Edge exclusion: reject blobs within 3% of frame border
        // Camera edge artifacts always appear near edges; real people don't
        const float EDGE = 0.03f;
        if (_tracks[t].normX < EDGE || _tracks[t].normX > 1.0f - EDGE ||
            _tracks[t].normY < EDGE || _tracks[t].normY > 1.0f - EDGE) continue;

        // Static blob rejection: if blob centroid hasn't moved in the last
        // 30 frames (tracked via accumulated position variance) skip it.
        // Uses velocity magnitude as proxy -- truly static = ~0 velocity
        float speed = sqrtf(_tracks[t].vx * _tracks[t].vx +
                            _tracks[t].vy * _tracks[t].vy);
        // Allow blobs that have moved recently OR are young (still arriving)
        bool isMoving = (speed > 0.005f || _tracks[t].age < 20);
        if (!isMoving) continue;

        float ageFactor  = fminf(1.0f, _tracks[t].age / 10.0f);
        float fresh      = 1.0f - (float)(now - _tracks[t].lastSeenMs) / blobTimeoutMs;
        _tracks[t].score = _tracks[t].area * 3.0f * ageFactor * fresh;
        _tracks[t].score = fminf(1.0f, _tracks[t].score);

        if (res.count < MAX_BLOBS)
            res.blobs[res.count++] = _tracks[t];
    }

    // Sort by score
    for (int i=0; i<res.count-1; i++)
        for (int j=i+1; j<res.count; j++)
            if (res.blobs[j].score > res.blobs[i].score) {
                Blob tmp = res.blobs[i];
                res.blobs[i] = res.blobs[j];
                res.blobs[j] = tmp;
            }

    res.anyMotion = (res.count > 0);
    return res;
}

// -- process -------------------------------------------------------------------
BlobResult BlobTracker::process(const uint8_t* grey, int w, int h) {
    uint32_t now = millis();

    // Periodic background reset (handles outdoor lighting changes)
    if (_bgReady && (now - _lastBgReset) > bgResetIntervalMs) {
        resetBackground();
    }

    _downsample(grey, w, h);

    // -- Background warmup ----------------------------------------------------
    // On first call: run 60 fast-learning iterations on this single frame
    // to get a solid background model before any detection starts.
    // bgr=5 (0.005) takes ~200 frames to converge -- warmup uses alpha=50 (0.05)
    // which converges 10x faster: 20 iterations gets to 64% of final value.
    if (!_bgReady) {
        // Seed avg buffers with this frame
        for (int i = 0; i < MAX_AVG; i++) {
            if (_avgBuf[i]) memcpy(_avgBuf[i], _workGrey, BT_PIX);
        }
        memcpy(_avgGrey, _workGrey, BT_PIX);
        _avgHead = MAX_AVG - 1; _avgFilled = MAX_AVG;

        // Fast background learning -- 30 passes at 10x normal rate
        for (int pass = 0; pass < 30; pass++) {
            for (int i = 0; i < BT_PIX; i++) {
                int bg = _bgModel[i];
                _bgModel[i] = (uint8_t)(bg + (50 * ((int)_workGrey[i] - bg)) / 1000);
            }
        }
        _bgReady     = true;
        _lastBgReset = now;
        _lastFrameMs = now;

        // Clear any stale tracks from previous session
        for (int t = 0; t < MAX_BLOBS; t++) _tracks[t] = {};
        Serial.println("[BlobTracker] Background initialised");
        return {};
    }

    _updateFrameAverage();   // rolling average -- cancels lamp flicker
    _computeMotion();

    // -- Camera-shift detection ------------------------------------------------
    // Count fraction of pixels with motion. A person covers <20% of frame.
    // A camera shift covers >60% of frame. Trigger auto-reset when this happens.
    {
        int motionCount = 0;
        for (int i = 0; i < BT_PIX; i++)
            if (_motionMask[i] > 0) motionCount++;
        float motionFraction = (float)motionCount / BT_PIX;

        if (motionFraction > shiftThreshold && _bgReady) {
            Serial.printf("[BlobTracker] Scene shift detected (%.0f%% motion) -> resetting background\n", motionFraction * 100.0f);
            resetBackground();
            return {};   // skip this frame entirely
        }
    }

    _morphOpen(2);           // radius 2 = removes ~4px noise specks
    _updateBackground();     // motion-gated -- only learns still pixels

    // Find raw blobs
    Blob raw[MAX_BLOBS * 2];
    int rawCount = _labelBlobs(raw, MAX_BLOBS * 2);

    _matchAndUpdate(raw, rawCount, now);
    _updateDebugThumbs(now);

    _lastFrameMs = now;
    return _buildResult(now);
}