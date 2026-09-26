# 3kg Sumo Robot — XMotion Mega V2

An autonomous mini-sumo / 3kg-class sumo robot built on the **XMotion Mega V2** controller. It finds and pushes the opponent using three MZ80 proximity sensors, stays inside the dohyo with two QTR-1A line sensors, and uses a Sharp IR distance sensor for close-range push detection. Six selectable fight strategies are chosen with the on-board DIP switches before the match starts.

---

## Hardware

| Component | Qty | Role |
|---|---|---|
| XMotion Mega V2 | 1 | Main controller + motor driver + DIP switches |
| MZ80 (E18-D80NK) IR proximity sensor | 3 | Opponent detection — left, front, right (digital, active LOW) |
| QTR-1A reflectance sensor | 2 | Dohyo edge (white line) detection — front left & front right |
| Sharp GP2Y0A41SK0F IR distance sensor | 1 | Close-range distance (4–30 cm), used by the *Flags* strategy |
| Worm gear DC motor, 160 RPM | 2 | Left / right drive |
| Battery (LiPo recommended) | 1 | Power |

Worm gear motors are self-locking, so the robot is hard to push backwards when stopped — this is what makes the *Sit* strategies effective.

---

## Pin Map

| Signal | Pin | Notes |
|---|---|---|
| MZ80 Left | D1 | ⚠️ Shares Serial TX — see Known Issues |
| MZ80 Front | D14 | |
| MZ80 Right | D12 | |
| QTR Right | A4 | Analog, white < `WHITE_THRESHOLD` |
| QTR Left | A5 | Analog |
| Sharp IR | A1 | Analog, via SharpIR library |
| Left motor DIR / PWM | D5 / D9 | |
| Right motor DIR / PWM | D13 / D10 | |
| DIP switch 1 / 2 / 3 | D6 / D7 / D8 | `INPUT_PULLUP`, ON = LOW |

---

## Software Setup

1. Install the **Arduino IDE**.
2. Install the **SharpIR** library by Giuseppe Masino (Library Manager → search "SharpIR").
3. Select the board: **Arduino Mega 2560** (the XMotion Mega V2 is Mega-compatible).
4. Check that `MODEL` matches your Sharp sensor:
   ```cpp
   #define MODEL SharpIR::GP2Y0A41SK0F
   ```
5. Upload the sketch.

---

## Starting a Match

1. Set the DIP switches to the strategy you want (table below).
2. Power on / reset the robot.
3. The robot waits **5 seconds** (competition start delay), reads the switches once, and prints the selected code to Serial.
4. The fight begins. Changing switches mid-match has no effect — reset to re-select.

---

## Strategy Selection (DIP Switches)

Codes are written as `DS1 DS2 DS3`, where `1` = switch ON.

| Code | Strategy | Summary |
|---|---|---|
| `000` | **Normal** | Basic tracking; drives forward when nothing is seen |
| `001` | **Searching** | Tracking + active sweep-and-advance search pattern |
| `010` | **Flags** | Gets past the opponent's flags/front, then pushes using the Sharp IR |
| `011` | **Flank** | Opens with an arc along the edge to hit the opponent from the side |
| `100` | **Sit and Search** | Stays in place, rotating to scan; attacks on detection |
| `101` | **Sit and Wait** | Completely still until the opponent comes into view |
| `110` / `111` | — | Unassigned, falls back to **Normal** |

### Rule that applies to every strategy
**The line always wins.** Whenever either QTR sensor sees white, the robot backs up for 300 ms and pivots away from the edge that was hit (right edge → pivot left, left edge → pivot right, both → back up and pivot left). The only exception is the first `FLANK_TIMEOUT_MS` of the *Flank* strategy.

### Normal (`000`)
- Right sensor → pivot right. Left sensor → pivot left. Front → drive forward.
- If the opponent was lost less than 300 ms ago, keep turning toward where it was last seen.
- If it has been gone longer, drive forward slowly.

### Searching (`001`)
Uses `ReactOrSearch()`:
- **Front has priority.** Front + right → curve right while charging; front + left → curve left; front only → straight charge.
- Side only → pivot toward that side.
- Nothing seen → non-blocking search pattern: sweep one way (`SWEEP_RIGHT_MS` / `SWEEP_LEFT_MS`), sweep back, advance for `ADVANCE_MS`, then repeat starting from the other side. The first sweep direction is biased toward where the opponent was last seen.

### Flags (`010`)
Designed for opponents with flags or a wide front that fool the side sensors.
- All three sensors see it → charge straight.
- Front + right → pivot right 225 ms, drive forward 200 ms, mark as `turned`.
- Front + left → mirror of the above.
- Once `turned`, keep pushing at full power while the front sensor sees the opponent **or** the Sharp IR reads ≤ `PUSH_DISTANCE` (6 cm).
- If the front target is lost after turning, reset and fall back to `ReactOrSearch()`.

### Flank (`011`)
- At the start, edge detection is disabled for `FLANK_TIMEOUT_MS` (900 ms) while the robot arcs along the border: veers inward when on white, outward when on black, effectively riding the edge line.
- As soon as any MZ80 sees the opponent → attack and switch permanently to **Searching**.
- After the timeout, edge protection is forced back on so the robot can never drive off blind.

### Sit and Search (`100`)
- Stays in place; attacks on any detection (pivot toward side, full charge on front).
- With nothing seen, runs `RunSearch(300)`: rotates one way 300 ms, then the other way 300 ms, stopping early if anything is detected. Alternates starting direction each time.

### Sit and Wait (`101`)
- Does not move at all until the opponent is detected, then reacts like Sit and Search. Relies on the worm gears to resist being pushed.

---

## Tuning Parameters

| Constant | Default | What it does |
|---|---|---|
| `WHITE_THRESHOLD` | 150 | QTR reading below this = white edge. Calibrate on your dohyo. |
| `speedDiff` | 1.133 | Right/left motor mismatch factor. Re-measure after changing wheels, gearing or battery. |
| `PIVOT_R_PWM` / `PIVOT_L_PWM` | 110 | Search pivot speed |
| `SWEEP_RIGHT_MS` / `SWEEP_LEFT_MS` | 270 / 300 | Search sweep durations |
| `ADVANCE_MS` | 350 | Forward step in the search pattern |
| `SEARCH_ADV_L` | 150 | Search advance speed (right side derived from `speedDiff`) |
| `PUSH_DISTANCE` | 6 cm | Sharp IR distance treated as "in contact" (Flags) |
| `SIT_DISTANCE` | 8 cm | Reserved for Sit and Wait (currently unused) |
| `FLANK_TIMEOUT_MS` | 900 | How long edge detection is ignored during the flank arc |

**Calibrating the line threshold:** open the Serial Monitor (9600 baud), place each QTR sensor over black and over the white border, and pick a value roughly halfway between the two readings.

---

## Code Structure

| Function | Purpose |
|---|---|
| `setup()` | Pin setup, 5 s start delay, DIP switch strategy selection |
| `loop()` | Reads sensors, applies line priority, dispatches to the active strategy |
| `BasicReact()` | Normal strategy logic with last-seen memory |
| `ReactOrSearch()` | Front-priority attack + non-blocking search fallback |
| `RestartSearch()` / `UpdateSearch()` | Non-blocking sweep/advance search state machine |
| `RunSearch()` | Blocking scan used by Sit and Search |
| `Onwhite()` | Edge escape manoeuvre |
| `DriveMotors()` | Signed PWM (−255…255) to both motors, with clamping |
| `pivotRight()` / `pivotLeft()` | In-place rotation helpers |

---

## Known Issues / TODO

- **Left MZ80 on D1** — D1 is the hardware Serial TX pin, and `Serial` is active. This can corrupt the left sensor reading. Move it to a free digital pin.
- **Heavy Serial output** — printing sensor tables every loop at 9600 baud slows the loop noticeably. Disable debug prints for competition or raise the baud rate.
- **Sit and Wait never stops** — after losing the opponent, the motors keep their last command (e.g. keep spinning). Add a `DriveMotors(0, 0)` in the "nothing detected" case.
- **Blocking code ignores the line** — `RunSearch()`, `Onwhite()` and the Flags turn use `delay()` / busy loops, during which edge sensors are not checked.
- `SIT_DISTANCE` is defined but not used yet.
- The `searching` flag in Sit and Search is redundant because `RunSearch()` is blocking.

---

## License

Add your license here (e.g. MIT).
