# EyeWatcher -- Development Context

This document records the key architectural decisions, bugs encountered, and rationale behind choices made during development. Intended as a reference for future maintenance and for understanding why things are the way they are.

---

## Project origin and concept

The installation places security camera housings on poles outdoors, each containing an animatronic human eye. A fixed wide-angle camera inside the housing watches the space; the eye follows people. The effect is a surveillance camera that watches back with a human gaze -- sleeping when the space is empty, waking when people arrive, tracking individuals one by one.

Three units were built and deployed for outdoor public installation.

---

## Architecture decisions

### Dual-core split (non-negotiable)

Camera processing runs on Core 0, everything else on Core 1. This is the single most important architectural decision and was established early.

**Why:** The ESP32-S3 camera driver uses DMA interrupts that need Core 0. If the camera task runs on Core 1 alongside the I2C servo writes, servo jitter occurs because I2C is a blocking operation that competes with camera DMA timing. Keeping the camera isolated on Core 0 eliminates this entirely.

**The rule:** Nothing blocking runs on Core 0. No Wire.begin(), no Serial during the task, no delay(). Core 0 does camera grab -> process -> post to queue -> repeat.

**Queue design:** `xQueueCreate(1, sizeof(BlobResult))` with `xQueueOverwrite()`. Queue depth 1 ensures the behaviour layer always reads the freshest detection result. If Core 1 is slow, it just misses frames -- it never builds up stale detections.

### Camera must init before Wire.begin()

The camera SCCB control bus uses I2C peripheral 1 internally. Wire.begin() initialises I2C peripheral 0. If Wire.begin() runs first, peripheral 1 is in an uncertain state and camera init fails silently (sensor PID is detected but frames don't arrive).

**Fix:** `cameraInit()` always runs before `Wire.begin()` in setup(). This ordering is fixed and must not be changed.

### Two-phase camera init (JPEG UXGA -> GRAYSCALE QVGA)

For OV3660 and OV5640, starting directly in GRAYSCALE QVGA mode causes `esp_camera_fb_get()` to return null with no error. The fix is to init as JPEG UXGA first, grab one frame (which sizes the DMA buffers correctly), then deinit and reinit as GRAYSCALE QVGA.

**Why UXGA first:** The DMA buffer sizing depends on the maximum frame size the sensor will ever produce. JPEG UXGA is the largest, so it establishes the correct allocation. QVGA after that fits easily within the already-allocated buffers.

**OV2640 is different:** OV2640 does NOT need this two-phase init. It works directly at any resolution. It also requires 10MHz XCLK (not 20MHz) and `CAMERA_GRAB_WHEN_EMPTY` (not `GRAB_LATEST`). Using the OV3660 settings with an OV2640 causes silent frame grab failures.

### Grayscale QVGA only

All processing uses 320x240 grayscale. No colour, no higher resolution, no JPEG decode.

**Why:** At 11fps, QVGA grayscale gives enough temporal resolution to track people walking. Higher resolution increases processing time without improving tracking quality. Grayscale eliminates the JPEG decode step and the colour demosaicing overhead. The camera does the downsampling and grayscale conversion in hardware, delivering raw bytes directly to PSRAM.

### Background model: motion-gated EMA

Background subtraction uses an exponential moving average (EMA) that only updates at pixels currently classified as still. Moving pixels are excluded from background learning.

**Why motion-gating:** Without gating, a slowly-moving object (person walking slowly) gets absorbed into the background model before it can be detected as motion. With gating, moving regions are preserved as motion indefinitely until they stop, at which point the background adapts to include them over a few seconds.

**The force-learn pass:** Every 60 frames, ALL pixels (including motion pixels) are updated at 1/10 the normal rate. This prevents a noise pixel that always triggers motion from being permanently excluded from the background model -- eventually it gets absorbed.

### Vision pipeline switch at runtime

Both blob tracker and saliency map are compiled in and can be switched with `blob`/`sal` serial commands without reboot. Both produce `BlobResult` so the behaviour layer is pipeline-agnostic.

**Why saliency was built first:** The original concept was a full saliency map (motion + colour + brightness + habituation), which worked indoors but was unreliable outdoors due to lighting variation. The blob tracker was added as a more robust alternative for outdoor use.

**Why keep saliency:** For indoor gallery contexts, the saliency map produces more artistically interesting behaviour -- it responds to colour and brightness changes as well as motion, and the habituation map creates a richer pattern of attention. The runtime switch lets the same hardware serve both contexts.

---

## Sensor/camera bugs and resolutions

### OV2640 false hardware fault diagnosis

An OV2640 module was initially assumed defective because `esp_camera_fb_get()` returned null even though the sensor PID (0x26) was correctly detected on SCCB.

**Root cause:** Software, not hardware. The OV2640 needs 10MHz XCLK and `CAMERA_GRAB_WHEN_EMPTY`. The code was using OV3660 settings (20MHz, `GRAB_LATEST`). A second OV2640 also appeared to fail for the same reason.

**Resolution:** Auto-detect sensor PID before main init, choose parameters accordingly. The `cam_test` diagnostic sketch was written to try all parameter combinations systematically, which confirmed the issue was settings not hardware.

**Lesson:** Always verify SCCB communication (PID detected) separately from DVP data communication (frame grab). PID detected + grab fails = software/timing, not wiring.

### `cam_hal: EV-VSYNC-OVF` messages

These appear frequently in logs, especially when WiFi is active.

**What it means:** The camera DMA buffer overflowed -- the ESP32 wasn't fast enough to consume a frame before the next VSYNC arrived. The camera driver handles this gracefully by dropping the frame.

**Not a problem:** Occasional overflow is normal. It causes a slight fps drop but no data corruption. It happens more when WiFi radio is active because WiFi interrupts compete with camera DMA timing. Suppressed by lowering `CORE_DEBUG_LEVEL` to 2 in `platformio.ini`.

---

## Background model bugs and resolutions

### Background model taking 18+ seconds to converge

**Symptom:** For the first 18+ seconds after boot or reset, the entire scene appeared as motion, creating many false blobs everywhere.

**Root cause:** Default `bgAlphaInt=5` (0.5% per frame). At 11fps, 63% convergence takes 200 frames = 18 seconds. During this time every pixel differs from the background and shows as motion.

**Fix 1:** Raised to `bgAlphaInt=15` (1.5%/frame), converging in ~65 frames (~6s).

**Fix 2:** Added a fast-warmup pass on init: 30 iterations at 10x speed on the first real frame. Background is 95% accurate after the first `process()` call.

**Fix 3:** Vision task collects 10 real frames into the averaging buffer before calling `resetBackground()`, so the fast-warmup uses a representative scene rather than a single potentially-atypical frame.

### Persistent noise pixels never absorbed into background

**Symptom:** Pixels with persistent flicker (electrical noise, lamp frequency beating with frame rate) never got absorbed into the background model because they always triggered the motion gate.

**Root cause:** Motion-gated background update permanently excluded noisy pixels. Once a pixel was classified as moving, it was never updated in the background model and remained classified as moving forever.

**Fix:** Force-learn pass every 60 frames: all pixels including motion-masked ones are updated at alpha=2 (1/10 normal rate). Slow enough not to corrupt the model for real motion, fast enough to eventually absorb persistent noise.

### `visionBgSettled` auto-reset loop

**Symptom:** System kept cycling DOZING -> WAKING because `visionBgSettled` kept flipping false.

**Root cause:** The settle check was calling `resetBackground()` every 4s whenever blob count > 1, which set `visionBgSettled=false`, which triggered another DOZING->WAKING transition, which had blobs, which triggered another reset. Infinite loop.

**Fix:** Removed the auto-reset from the settle check. `visionBgSettled` is set true after 4 seconds since last reset, period. The background EMA handles ongoing convergence; the settle flag is just a startup gate.

### Camera shift detection triggering on noise

The shift detection compares raw frame to background and triggers a reset when >threshold fraction of pixels differ. Originally set at 0.45 (45%), this triggered on normal activity when several blobs were present simultaneously, or during the post-reset warmup period when the background model wasn't accurate yet.

**Fix:** Added a 5-second cooldown after any shift detection fires. The threshold itself stays at 0.45 -- this is correct for distinguishing a genuine camera shift from people moving. What was wrong was re-triggering repeatedly during the post-shift recovery period.

---

## Behaviour state machine bugs and resolutions

### SLEEPING -> WAKING with no blobs (persistent)

**Symptom:** Eye refused to stay asleep. Would fall asleep correctly but immediately wake up again, even with camera pointed at empty wall.

**Root causes (multiple, resolved in sequence):**

1. **`initialWake` condition** -- `_doSleeping()` had `bool initialWake = (now - _stateMs) < 1000` as a boot condition, but `_stateMs` was being reset every time the state was re-entered, so it fired on every entry to SLEEPING, not just the first boot. Fixed by adding `_isFirstBoot` flag.

2. **Stale blob tracks with high scores** -- Blob tracks persist for `blobTimeoutMs` (800ms) after last being seen. A blob seen while SCANNING that then disappeared still had a non-zero score for up to 800ms. During that window, `_doSleeping()` saw `blob.score > 0.15` and woke up. Fixed by requiring `speed > 0.005` in `_buildResult` -- static blobs (stopped people, expiring tracks with zero velocity) are not reported.

3. **`age < 20` exemption** -- The velocity check had `speed > 0.005f || _tracks[t].age < 20` which exempted any blob younger than 20 frames from the velocity requirement. A blob created when you walked past remained valid for 20 more frames (1.8s) even after you stopped, keeping the eye awake. Removed the age exemption entirely.

### Eye always tracking the same blob after sleeping

**Symptom:** After waking from sleep, eye immediately committed to an edge-of-frame artifact and stayed there.

**Root cause 1:** The post-wake background reset was happening before the lids had opened. The first real scene (lids closed, dark) was being learned as the background, so when the lids opened the whole scene appeared as motion.

**Fix:** `scheduleBackgroundReset(1500)` is called from `_doWaking()` at 1.5 seconds into waking (not from `_doSleeping()`). By 1.5s the lids are mostly open and the camera sees the actual scene.

**Root cause 2:** Frame edge artifacts. Camera sensors produce edge artifacts from lens vignetting and DVP signal ringing. These appear as persistent motion at the extreme edges of frame.

**Fix:** Edge exclusion zone -- blobs with normX < 0.03 or > 0.97, or normY < 0.03 or > 0.97 are rejected in `_buildResult`.

### Blink continuing during SLEEPING

**Symptom:** Eye blinked even while in SLEEPING state (lids should be fully closed).

**Root cause:** `_handleBlink()` correctly refused to START a new blink when sleeping, but it couldn't cancel a blink already IN PROGRESS. The blink animation in `servo_eye.cpp:update()` had no state check and would complete regardless of what the state machine set for arousal.

**Fix:** `setSleeping()` and `_doScared()` now call `cancelBlink()` which sets `_blinking = false`. The update loop then abandons the animation immediately and lets the sleeping arousal (0.0) take over.

### Dwell time never executing

**Symptom:** Despite 3-8 second dwell timers, the eye was jittering with new targets every few hundred milliseconds.

**Root cause:** The dwell logic checked `if (sal.numPeaks > 1)` to decide whether to shift. But the queue only ever carried one peak (the current winner), so `numPeaks` was always 1 and the shift condition never fired. The eye was gazing at `sal.normX/Y` which changed every frame as the saliency centroid shifted.

**Fix:** Introduced `_committedX/Y` -- the behaviour layer picks a target, commits to those coordinates, and ignores new saliency results until the dwell expires. The saliency pipeline keeps running and habituating the committed point, but the gaze doesn't follow it frame-by-frame.

### Calibration pan center not moving servo

**Symptom:** Typing `pan` then `center` then `++` changed the displayed value but the servo didn't move.

**Root cause 1:** Axis selection code read from `data.panCenter` but the nudge command wrote to `work.panCenter`. After the work/data split was introduced, these were no longer the same reference.

**Root cause 2:** `_panTiltPos` was never initialised when `pan` was typed -- it remained at whatever it was from the previous calibration session. If it was MAX from last time, nudging wrote to `work.panMax` not `work.panCenter`.

**Fix:** Axis selection commands (`pan`, `tilt`, `top`, `bot`) now explicitly set `_panTiltPos = PanTiltPos::CENTER` and read initial position from `work` not `data`.

### Calibration data vs work split

**Problem:** Calibration edits were being written immediately to the live `data` struct which is used by the servo_eye at runtime. Any nudge immediately changed the running eye behavior with no way to preview or discard.

**Fix:** Two-struct model:
- `data` -- committed values, written to NVS on `save`, used by servo_eye at runtime
- `work` -- editing scratchpad, initialised from `data` at start of cal session

All nudges, axis selections, and preview commands operate on `work`. Status shows `* UNSAVED *` when work differs from data. `save` validates work with `_isValid()` then commits: `data = work`, then writes NVS. `exit` without save discards work silently (with a warning if dirty).

---

## WebSocket / web UI bugs and resolutions

### WebSocket queue flooding

**Symptom:** `[AsyncWebSocket] Too many messages queued: discarding` filling the serial log, connection dropping.

**Root cause:** Sending a full JSON frame (status + base64-encoded motion mask + blob map) at 5fps. The frame size was ~3.5KB. The async WebSocket library has a fixed internal queue depth -- when the WiFi radio was briefly busy the queue filled and started dropping, then the client detected missed frames and reconnected, creating a reconnection loop.

**Fix 1:** Split into two frame types:
- Status frame (~250 bytes): state, pan, tilt, arousal, blobs, log -- sent every 1 second
- Canvas frame (~900 bytes): motion mask + blob map -- sent every 3 seconds OR on client request

**Fix 2:** Client sends `"canvas"` WebSocket message to request an immediate canvas refresh, avoiding the need for frequent server-push.

**Fix 3:** Downsampled canvas from 40x30 to 20x15 (4x smaller payload) before base64 encoding.

**Fix 4:** `if (ws.count() == 0) return` at top of `webUiLoop()` -- skip all serialisation work when no client is connected. This matters during deployment when nobody is monitoring.

### em-dash characters in string literals

**Symptom:** Compiler error `missing terminating " character` with no obvious cause.

**Root cause:** Unicode em-dash character (U+2014, bytes `e2 80 94`) was generated in `Serial.printf()` string literals. The ESP32 Arduino compiler treats source as ASCII -- any multi-byte UTF-8 character in a string literal breaks parsing.

**Fix:** Replaced all em-dashes with `--` or `->` in string literals. Non-string uses (comments, box-drawing) are fine. A binary scan was run across all source files to find remaining multi-byte sequences.

**Lesson:** Never use Unicode typographic characters (em-dash, curly quotes, arrows, box-drawing) inside `Serial.printf()` strings. Comments are safe; string literals are not.

---

## Saliency pipeline notes

The saliency pipeline was the original approach and works well indoors but proved unreliable outdoors because:
- Colour saliency fires on sky/cloud changes (large illuminated areas)
- Brightness saliency fires on shadows from clouds
- The habituation map takes too long to suppress these

The blob tracker was built as a replacement for outdoor use. Key differences:

- Blob tracker uses ONLY motion (background subtraction) as the signal
- No colour, no brightness -- immune to lighting variation
- Blob identity is tracked across frames (persistent IDs) -- enables round-robin attention between multiple people
- Blob habituation is per-track (static people get boring) not per-pixel

Both remain compiled in and switchable at runtime.

---

## Blob habituation model

Static blobs (people who stop moving) should lose the eye's interest and eventually cause the eye to sleep. The implementation uses a per-track `habit` field (0.0-1.0).

Each frame a blob is matched:
- If blob velocity < 0.01 (stationary): `habit += habitStrength` (default 0.008/frame)
- If blob is moving: `habit *= habitDecay` (default 0.995/frame, fast decay)

In `_buildResult`, a blob is excluded from output if `speed <= 0.005 AND habit >= habitThreshold` (default 0.7). A stationary blob reaches the threshold in approximately `0.7 / 0.008 = 87 frames` (~8s at 11fps).

When a person moves again after being habituated, their `habit` value decays quickly (0.995^n) and they become salient again within a few seconds.

---

## Camera coordinate mapping

The camera's field of view is wider than the eye's servo travel range. A person at the extreme left of the camera frame should move the eye to its leftmost position, not overshoot.

Parameters `camPanScale` and `camTiltScale` define the ratio:
```
camPanScale = (eye servo range in degrees) / (camera FOV in degrees)
```
For a 120-degree wide-angle camera with 60-degree servo range: `campan 0.5`

All mapping parameters are runtime-tunable without recompile:
- `campan <f>` / `camtilt <f>` -- scale
- `camoffx <f>` / `camoffy <f>` -- offset (for off-center camera mounting)
- `camflipx` / `camflipy` -- mirror axes

These are implemented as `extern float` variables defined in `main.cpp` and referenced in `servo_eye.cpp::_normToPan()` / `_normToTilt()`.

---

## Shift detection

When the installation pole is bumped or wind causes vibration, the background model is instantly wrong and every pixel shows as motion. The blob tracker fires a `blobTrackerShiftDetected` flag which the behaviour layer catches and transitions to SCARED state (lids shut, eye centers, wait for scene to settle).

Detection method: compare raw current frame (bypassing frame averaging) to background model. If fraction of differing pixels > `shiftThreshold` (default 0.45), it's a shift.

**Why raw frame not averaged:** The frame averaging buffer dilutes a sudden scene change across 3-4 frames, so a 100% shift might only show as 30-40% motion on the first frame. Using the raw frame gives the full signal immediately.

**5-second cooldown:** After a shift fires, it's suppressed for 5 seconds. During post-shift recovery the background model is re-learning and may briefly show high motion fractions that would re-trigger without the cooldown.

**Threshold choice (0.45):** People moving cover < 20% of pixels. A camera shift covers 70-100%. The 0.45 threshold has comfortable margin from both. Raise to 0.55-0.60 if wind causes the pole to vibrate enough to trigger false shifts outdoors.

---

## SCARED state rationale

The SCARED state was added specifically to handle camera shifts gracefully in a public installation context.

The artistic intent: when the installation is physically disturbed (bumped, pole vibrates in wind), the eye should react with a visible fear response -- lids snap shut, eye centers, holds closed -- rather than thrashing randomly as the background model rebuilds.

Recovery condition: `elapsed > 3s AND visionBgSettled AND blobs.count <= 1`. The `blobs <= 1` requirement means the eye won't reopen into a chaotic scene with many artifacts. It waits until the world makes sense again (at most one thing moving).

The recovery path goes through WAKING (not directly to SCANNING), repeating the slow lid-opening sequence with a fresh background reset, so the first thing the eye sees after recovering is a clean well-learned scene.

---

## WiFi and networking

### Captive portal

iOS/Android detect captive portals by making HTTP requests to known URLs (`/generate_204`, `/hotspot-detect.html`, etc.). If these return anything other than their expected response, the OS shows a "sign in to network" notification.

Our server redirects all unknown URLs to `http://192.168.4.1/` which causes the OS to open the browser automatically. A DNS server (`DNSServer`) runs on port 53 and redirects all domain queries to `192.168.4.1`, which triggers the captive portal detection before the HTTP redirect.

### Channel selection

In busy WiFi environments (galleries, public spaces) the default channel may be congested. The channel is set in `platformio.ini` as `-DWIFI_CHANNEL=11` so each unit can be configured independently. Channels 1, 6, and 11 are the standard non-overlapping 2.4GHz channels and are typically the most congested. Try channels 3, 8, or 13 for less interference.

### WebSocket vs HTTP polling

WebSocket was chosen over HTTP polling for the web UI because:
- Server can push state changes immediately without client polling
- Lower per-frame overhead (no HTTP headers)
- Persistent connection allows the status bar to show connection state

The downside is that WebSocket connections are stateful and can drop, requiring reconnection logic in the client. The HTML UI handles this with a 2-second reconnect timer.