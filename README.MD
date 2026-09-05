# Hexapod Control Manual
## Using UDP Commands to Control a Hexapod Robot

**Date:** March 004, 2025  
**Firmware Version:** Based on Hexapod_Input.cpp dated February 27, 2025  
**Developed by:** Erik Lely (with inspiration from the original code)

This manual provides instructions for controlling a hexapod robot via WiFi using UDP packets. The hexapod listens for commands on UDP port 8888 and sends responses or debug information on UDP port 8889. Commands are sent in a binary packet format, and this guide explains how to construct and use these packets effectively.

---

## 1. Overview

The hexapod robot is controlled by sending UDP packets containing commands to move its body, legs, or execute special actions (e.g., stop, halt, or power off). The robot supports multiple control modes (walking, body movement, mixed mode) and responds to joystick-like inputs for axes and button presses.

### Key Features
- **UDP Ports:**
  - Receive Commands: Port 8888
  - Send Responses: Port 8889
- **Control Inputs:** 6 axes (left X/Y, right X/Y, secondary X/Y) and 13 buttons
- **Modes:** Walking, body movement, mixed mode, and no mode
- **Special Commands:** Stop, halt, and power off the robot

---

## 2. Command Packet Structure

Commands are sent as binary packets with the following structure (defined as `CommandPacket` in the code):

| Field           | Type     | Size (Bytes) | Description           | Valid Values                  |
|-----------------|----------|--------------|-----------------------|-------------------------------|
| `magic`         | `uint32_t` | 4          | Validation marker     | Must be `0xDEADBEEF`          |
| `version`       | `uint8_t`  | 1          | Protocol version      | Must be `1`                   |
| `special_command`| `uint8_t`  | 1          | Special command type  | `0` (normal), `1` (STOP), `2` (HALT), `3` (POWER) |
| `axes_lx`       | `int16_t`  | 2          | Left X axis (lateral) | -32768 to 32767 (clamped to -512 to 512) |
| `axes_ly`       | `int16_t`  | 2          | Left Y axis (fwd/bwd) | -32768 to 32767 (clamped to -512 to 512) |
| `axes_rx`       | `int16_t`  | 2          | Right X axis (rot)    | -32768 to 32767 (clamped to -512 to 512) |
| `axes_ry`       | `int16_t`  | 2          | Right Y axis (height) | -32768 to 32767 (clamped to -512 to 512) |
| `axes_sx`       | `int16_t`  | 2          | Secondary X axis (Z rot) | -32768 to 32767 (clamped to -512 to 512) |
| `axes_sy`       | `int16_t`  | 2          | Secondary Y axis (X rot) | -32768 to 32767 (clamped to -512 to 512) |
| `buttons`       | `uint16_t` | 2          | Bitfield for buttons  | 0 to 0x1FFF (13 bits used)    |
| `duration_ms`   | `uint32_t` | 4          | Duration of movement (ms) | 0 to 4294967295           |

**Total Size:** 24 bytes  
**Note:** The packet must be packed tightly (no padding), as indicated by `__attribute__((packed))` in the code.

### Button Bitfield (`buttons`)
Each bit corresponds to a specific button in this 16-bit `uint16_t` (13 bits used):

- **Bit 0:** `button_start` - Toggle power on/off
- **Bit 1:** `button_mode_select` - Cycle control modes (WALKMODE, BODYMOVEMODE, MIXEDMODE, NOMODE)
- **Bit 2:** `button_gait_select` - Cycle gait types when stationary
- **Bit 3:** `button_l3` - Toggle debug output to port 8889
- **Bit 4:** `button_r2` - Toggle walk method in MIXEDMODE
- **Bit 5:** `button_r3` - Adjust leg positions to body height
- **Bit 6:** `button_balance_mode` - Toggle balance mode
- **Bit 7:** `button_updown_select` - Stand up or sit down
- **Bit 8:** `button_camera_mode` - Adjust body height, speed, or leg angles
- **Bit 9:** `button_values_reset` - Reset all positions to default
- **Bit 10:** `button_attack_mode` - Enter attack stance
- **Bit 11:** `button_travel_select` - Toggle travel height/speed
- **Bit 12:** `button_single_leg` - Cycle single-leg control (legs 0-5, then off)
- **Bits 13-15:** Unused - Reserved for future expansion

**Valid Range:** 0 to 0x1FFF (13 bits)

---

## 3. Sending Commands

### Prerequisites
- **Network:** The hexapod must be on the same WiFi network as the controlling device.
- **IP Address:** Identify the hexapod’s IP address (e.g., via a network scan or predefined configuration).
- **UDP Client:** Use a tool or program (e.g., Python with `socket`, a custom C# app) to send UDP packets.

### Steps
1. **Construct the Packet:**
   - Set `magic` to `0xDEADBEEF` and `version` to `1`.
   - Choose a `special_command` (0 for normal, 1-3 for special actions).
   - For `special_command = 0`, set axis values (-32768 to 32767, scaled internally) and `buttons` as a bitfield.
   - Set `duration_ms` for sustained movement (0 for immediate stop).
2. **Send the Packet:**
   - Send the 24-byte binary packet to the hexapod’s IP address on port 8888.
3. **Receive Responses:**
   - Listen on port 8889 for debug messages or status updates (e.g., "INFO: Hexapod on" or "ERROR: Invalid packet").

---

## 4. Command Descriptions

### Special Commands (`special_command`)
- **`0` - Normal Command:**
  - Uses axis and button inputs to control movement, modes, and actions.
  - Axes control walking (WALKMODE), body position (BODYMOVEMODE), or both (MIXEDMODE).
  - Buttons trigger mode changes, resets, or actions (see below).
  - `duration_ms` sustains the movement.
- **`1` - STOP:**
  - Turns off the hexapod, stopping all servo activity and resetting its state.
  - Axes and buttons ignored.
  - Emergency stop or end operation.
- **`2` - HALT:**
  - Stops sustained movement (resets `sustained_axes_*` to 0).
  - Robot remains powered and ready for new commands.
  - Pauses motion without affecting power or mode.
- **`3` - POWER:**
  - Shuts down the system (`sudo shutdown -h now`).
  - Requires physical restart.
  - Administrative shutdown; axes/buttons ignored.

### Axis Controls
- **Scaled Range:** -512 to 512 (after clamping and scaling)
- **Modes:**
  - **WALKMODE:**
    - `axes_lx`: Lateral movement (left/right)
    - `axes_ly`: Forward/backward movement
    - `axes_rx`: Rotation (yaw)
    - `axes_ry`: Single-leg height (if selected)
  - **BODYMOVEMODE:**
    - `axes_lx`: Body X position
    - `axes_ly`: Body Z position
    - `axes_rx`: Body rotation Y
    - `axes_ry`: Body Y shift
    - `axes_sx`: Body rotation Z
    - `axes_sy`: Body rotation X
  - **MIXEDMODE:**
    - Combines walking and body movement (see code for specifics)

### Button Commands (for `special_command = 0`)
- `button_start` (Bit 0): Toggles hexapod on/off
- `button_mode_select` (Bit 1): Cycles modes (WALKMODE → BODYMOVEMODE → MIXEDMODE → NOMODE)
- `button_gait_select` (Bit 2): Cycles gaits (e.g., Ripple 12, Tripod 8) when stationary
- `button_l3` (Bit 3): Toggles debug output (port 8889)
- `button_r2` (Bit 4): Toggles walk method in MIXEDMODE
- `button_r3` (Bit 5): Adjusts leg positions to body height
- `button_balance_mode` (Bit 6): Toggles balance mode
- `button_updown_select` (Bit 7): Stands up or sits down
- `button_camera_mode` (Bit 8): Adjusts height (`axes_ry`), speed (`axes_rx`), or angles (`axes_lx`, `axes_ly`)
- `button_values_reset` (Bit 9): Resets positions/settings to default
- `button_attack_mode` (Bit 10): Sets attack stance
- `button_travel_select` (Bit 11): Toggles travel height/speed
- `button_single_leg` (Bit 12): Cycles single-leg control (legs 0-5, off)

---

## 5. Example Usage

### Example 1: Move Forward
- **Packet:**
  - `magic`: `0xDEADBEEF`
  - `version`: `1`
  - `special_command`: `0`
  - `axes_lx`: `0`
  - `axes_ly`: `512` (forward)
  - `axes_rx`: `0`
  - `axes_ry`: `0`
  - `axes_sx`: `0`
  - `axes_sy`: `0`
  - `buttons`: `0`
  - `duration_ms`: `1000` (1 second)
- **Effect:** Hexapod walks forward for 1 second.

### Example 2: Stop the Hexapod
- **Packet:**
  - `magic`: `0xDEADBEEF`
  - `version`: `1`
  - `special_command`: `1`
  - All other fields: `0`
- **Effect:** Hexapod turns off immediately.

### Example 3: Halt Movement
- **Packet:**
  - `magic`: `0xDEADBEEF`
  - `version`: `1`
  - `special_command`: `2`
  - All other fields: `0`
- **Effect:** Sustained movement stops; hexapod remains powered.

### Example 4: Power Off the System
- **Packet:**
  - `magic`: `0xDEADBEEF`
  - `version`: `1`
  - `special_command`: `3`
  - All other fields: `0`
- **Effect:** Hexapod shuts down after a 6-second delay.

---

## 6. Debugging and Feedback
- **Port 8889 Responses:**
  - Messages prefixed with `INFO:` (normal status) or `ERROR:` (issues).
  - Examples: `INFO: STOP command`, `ERROR: Invalid packet`.
- **Enable Debug:**
  - Send `button_l3` (Bit 3) with `special_command = 0` to enable verbose output.
- **Timeouts:**
  - If no commands are received for `TIMEOUT_MS`, the hexapod stops moving.

---

## 7. Safety Notes
- **STOP:** Use for immediate cessation; does not shut down the system.
- **HALT:** Pauses movement without powering off.
- **POWER:** Use cautiously; requires physical restart.
- **Timeouts:** Send continuous commands to avoid unintended stops.
- **Clamping:** Axis values are clamped to prevent extreme movements.
