# Hexapod optimizations (RPi 5)

Notes on optimizations applied in the hexapod codebase for speed and reduced syscalls on Raspberry Pi 5.

---

## Applied optimizations

### 1. Flush stdout only when printing (`Hexapod_Code.cpp`)

**What:** In `loop()`, `fflush(stdout)` is now called only when `g_fPrintEnabled` is true.

**Why:** When debug printing is disabled, calling `fflush(stdout)` every loop iteration added an unnecessary syscall. Skipping it in the common case reduces overhead.

---

### 2. Optional no-op for PRINTUDP (`Hex_Cfg.h`)

**What:** `PRINTUDP` can be compiled out by defining `HEXAPOD_DISABLE_UDP_ECHO` in the compiler flags (e.g. `-DHEXAPOD_DISABLE_UDP_ECHO` in the makefile). When defined, `PRINTUDP(...)` expands to a no-op.

**Why:** Every `PRINTUDP(...)` call runs formatting and optional UDP send. For release builds where UDP echo to the controller is not needed, disabling it avoids `vsnprintf`/`strstr`/`sendto` in the hot path.

**How:** Add to your build (e.g. in the hexapod makefile):

```make
CFLAGS += -DHEXAPOD_DISABLE_UDP_ECHO
```

By default (without the define), behavior is unchanged: `PRINTUDP` still calls `sendUdpOnly`.

---

### 3. Pre-allocated AVFrame (`Hexapod_rpicam.cpp`)

**What:** The AVFrame used for BGR→YUV420P conversion is now allocated once (static) instead of `av_frame_alloc()` + `av_frame_free()` on every frame.

**Why:** Eliminates ~60 heap alloc/free per second from the camera thread, reducing latency jitter.

---

### 4. Zero-copy NV12 (`Hexapod_rpicam.cpp`)

**What:** The NV12 frame data is wrapped directly from the mmap'd camera buffer instead of being memcpy'd into a new `cv::Mat`.

**Why:** Saves ~115 KB memcpy per frame (~3.3 MB/s at 30 fps).

**Note:** Requires contiguous NV12 planes (standard for Pi cameras). If the camera driver uses non-contiguous planes, revert to the copy approach (see `_v1` backup).

---

### 5. Static BGR Mat (`Hexapod_rpicam.cpp`)

**What:** The BGR `cv::Mat` used for face detection/rendering is now pre-allocated once (static) instead of per-frame.

**Why:** Eliminates a ~230 KB heap allocation per frame.

---

### 6. UDP packet vector reserve (`Hexapod_rpicam.cpp`)

**What:** H.264 packet vectors now use `reserve()` before `assign()` to hint the allocator about the expected size.

**Why:** Reduces vector reallocation overhead for each encoded frame.

---

### 7. DNN skip frames (`Hexapod_rpicam.cpp`)

**What:** DNN face detection now runs every Nth frame (configurable via `DNN_SKIP_FRAMES`, default: 2). On skipped frames, the last detection result is reused.

**Why:** DNN inference is the heaviest operation in the camera thread. Running it every 2nd frame halves CPU load with minimal tracking impact.

---

### 8. Reduced DoBackgroundProcess calls (`Hexapod_Code.cpp`)

**What:** Removed `DoBackgroundProcess()` calls from inside the BalCalcOneLeg and BodyFK/LegIK math loops (6 calls removed).

**Why:** These loops are pure math with no meaningful time gap between iterations. The calls added unnecessary serial bus overhead without benefit.

---

### 9. Conditional main loop delay (`Hexapod_Code.cpp`)

**What:** The unconditional `delay(20)` at the end of `loop()` is now conditional — only applied when the robot is off.

**Why:** When the robot is active, the loop is already paced by `PrevServoMoveTime`. The extra 20 ms delay stacked on top, hurting responsiveness.

---

### 10. Integer square root (`Hexapod_Code.cpp`)

**What:** `isqrt32()` now uses integer Newton's method instead of `sqrt()` (floating-point).

**Why:** Avoids `int→double→sqrt→long` conversion. Called 12+ times per loop cycle from IK/trig functions.

---

### 11. Skip IdleTime during walk (`Hexapod_Code.cpp`)

**What:** `ServoDriver::IdleTime()` (which cycles through servo LEDs) is now skipped during active walking.

**Why:** Writing LED registers to each servo in round-robin wastes serial bus bandwidth during gait execution when the bus is busy with position updates.

---

### 12. sendUdpOnly early exit (`Hexapod_Input.cpp`)

**What:** `sendUdpOnly()` now exits immediately if UDP socket is not ready, and skips all string formatting/scanning if the message won't be sent (debug disabled and no "command" keyword).

**Why:** Eliminates `vsnprintf`, 6× `strstr`, and 2× `snprintf` calls on every `PRINTUDP(...)` in the hot path when they would be discarded anyway.

---

### 13. DEFINE_HEX_GLOBALS ownership clarified (`Hexapod_Code.cpp` / `Hexapod_Main.cpp`)

**What:** `#define DEFINE_HEX_GLOBALS` is now exclusively defined in `Hexapod_Code.cpp` and removed from `Hexapod_Main.cpp`.

**Why:** `Hex_Cfg.h` uses this macro to decide whether to emit the *definitions* of `g_abHexIntXZ` and `g_abHexMaxBodyY` (with actual storage) or just `extern` declarations. Since each `.cpp` is compiled independently, the macro must be present in exactly **one** translation unit — whichever file owns the global state. `Hexapod_Code.cpp` is the correct owner, as it already defines all other hexapod globals.

**⚠️ Previous mistake:** An earlier optimization attempt removed the define from `Hexapod_Code.cpp` under the incorrect assumption that it was "already defined in `Hexapod_Main.cpp`". That caused a linker error: `undefined reference to g_abHexMaxBodyY / g_abHexIntXZ`. Fixed by restoring the define to `Hexapod_Code.cpp` and removing it from `Hexapod_Main.cpp`.

---


## Other optional steps for RPi 5

- **Compiler flags:** Use `-O2 -march=armv8.2-a+crypto -mtune=cortex-a76` for release. Consider `-DNDEBUG` if you do not rely on `assert` in production.
- **Return-by-value trig:** Refactor `GetSinCos`, `GetATan2`, `GetArcCos` to return results via struct instead of globals. This enables compiler register optimization.
- **Integer SmoothControl:** Consider an integer version of `SmoothControl()` to avoid `int→double→double math→long` conversions.
- **Common scaling function:** The dynamic scaling code in WALKMODE, BODYMOVEMODE, and MIXEDMODE is duplicated. Extract into a shared function.

---

*Keep this file updated when further optimizations are applied.*
