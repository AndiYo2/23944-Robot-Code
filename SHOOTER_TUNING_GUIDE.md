# Shooter Tuning Guide — Team 23944 Decode

Complete reference for calibrating and troubleshooting the physics-based shooter calculator.

---

## Table of Contents

1. [First-Time Calibration (Step by Step)](#1-first-time-calibration-step-by-step)
2. [Pre-Match Verification Checklist](#2-pre-match-verification-checklist)
3. [Troubleshooting: Stationary Shooting](#3-troubleshooting-stationary-shooting)
4. [Troubleshooting: Shoot-While-Moving](#4-troubleshooting-shoot-while-moving)
5. [Troubleshooting: Flywheel](#5-troubleshooting-flywheel)
6. [Troubleshooting: Hood](#6-troubleshooting-hood)
7. [Troubleshooting: Turret](#7-troubleshooting-turret)
8. [Troubleshooting: System-Level](#8-troubleshooting-system-level)
9. [Quick-Reference Cheat Sheet](#9-quick-reference-cheat-sheet)
10. [Constants Reference](#10-constants-reference)

---

## 1. First-Time Calibration (Step by Step)

Complete this entire procedure before using shoot-while-moving. Do each step in order — later steps depend on earlier ones.

### Step 1: Verify Physical Measurements

**What you need**: Tape measure, level surface, robot powered off.

1. Place the robot on a flat tile surface.
2. Measure from the tile to the **center of the ball** as it sits in the exit position of the flywheel channel. This is `LAUNCHER_HEIGHT`.
   - Current value: **14.22 inches (361.2mm)**
   - If this changes (rebuild, new mounting), update `SCORE_HEIGHT = 38.75 - LAUNCHER_HEIGHT`
3. Verify the hood servo range:
   - Command the hood to `HOOD_MIN_ANGLE` (30 deg). Physically measure the angle from vertical with a protractor or phone inclinometer. Record actual angle.
   - Command the hood to `HOOD_MAX_ANGLE` (63 deg). Measure again.
   - If the physical angles don't match the code values, update `HOOD_MIN_ANGLE`, `HOOD_MAX_ANGLE`, and the servo calibration points (`HOOD_SERVO_AT_MIN_ANGLE`, `HOOD_SERVO_AT_MAX_ANGLE`).
4. Verify turret center: Command turret to 0 deg. It should point straight forward. If not, adjust `SERVO_CENTER_POSITION`.

**Done when**: All physical measurements match code constants.

---

### Step 2: Validate the LUT (Stationary Baseline)

**What you need**: Open field, balls, Panels dashboard connected. `SHOOT_WHILE_MOVING_ENABLED = false`.

1. Place the robot at a known position facing the goal (use tape marks).
2. Enable Panels telemetry to see: `distance`, `requiredVelocity`, `requiredHoodAngle`, `currentVelocity`, `turretAngle`.
3. Test at each LUT distance: **40, 60, 80, 100, 125, 140, 158 inches**.

For each distance:
   1. Position the robot so the turret-to-goal distance matches.
   2. Wait for the flywheel to reach target velocity (check `isAtTargetVelocity`).
   3. Shoot 5 balls. Record: how many score, miss direction (high/low/left/right).
   4. If <4/5 score, note the distance and miss pattern.

**Adjustments**:
- If all shots at a distance are slightly off, adjust that LUT entry.
- The user noted the LUT needs **+20 ticks/sec** globally. Apply this if shots consistently fall short.
- After adjustments, re-test.

**Done when**: 4/5+ shots score at every test distance.

---

### Step 3: Calibrate VELOCITY_CONVERSION_FACTOR (K)

**What you need**: Calculator or spreadsheet, the validated LUT from Step 2.

This converts between flywheel ticks/sec and actual ball exit velocity (in/s). Two methods:

#### Method A: Back-Calculate from LUT (Recommended — No Special Equipment)

For each LUT entry `(distance, ticks, hoodAngle)`:

```
launchAngle = 90 - hoodAngle                     (degrees, convert to radians)
x = distance                                      (inches)
y = SCORE_HEIGHT                                  (24.53 inches)
g = 386.088                                       (in/s^2)

v0 = sqrt( g * x^2 / (2 * cos^2(launchAngle) * (x * tan(launchAngle) - y)) )

K = v0 / ticks
```

Calculate K for at least 4 distances. Average them.

**Expected result**: K ~ 0.100-0.102

| Distance | Hood Angle | Ticks | Launch Angle | v0 (in/s) | K |
|----------|-----------|-------|--------------|-----------|-------|
| 40 | 32 | 1660 | 58 | ~167 | ~0.101 |
| 60 | 41 | 1880 | 49 | ~190 | ~0.101 |
| 80 | 45 | 2100 | 45 | ~211 | ~0.100 |
| 100 | 47 | 2180 | 43 | ~229 | ~0.105 |
| 125 | 51.5 | 2540 | 38.5 | ~256 | ~0.101 |
| 158 | 57 | 2860 | 33 | ~296 | ~0.104 |

**Note**: K values above vs below 45 deg hood angle may differ slightly (~0.100 vs ~0.103) due to the ball contact geometry. If precision matters, use two K values:
```java
static double K_BELOW_45 = 0.100;  // Hood angles 30-45 deg
static double K_ABOVE_45 = 0.103;  // Hood angles 45-63 deg
```

#### Method B: Range Test (Direct Measurement)

1. Set `TUNING_MODE = true` in Panels.
2. Set a known hood angle (e.g., 45 deg = 45 deg launch from horizontal) and velocity (e.g., 2100 ticks/s).
3. Place robot on flat ground, shoot into open space. Measure horizontal distance where ball first hits the ground.
4. Calculate:
   ```
   launchAngle = 90 - hoodAngle = 45 deg
   R = measured range (inches)
   launcherHeight = 14.22 inches (ball starts above ground)
   g = 386.088

   // Time to fall from launcher height at the end of the range:
   // Full calculation accounting for launch height:
   // R = v0 * cos(a) * t
   // 0 = launcherHeight + v0 * sin(a) * t - 0.5 * g * t^2
   // Solve quadratic for t, then v0 = R / (cos(a) * t)

   // Simplified (if launch height << range):
   v0 = sqrt(R * g / sin(2 * launchAngle))
   K = v0 / ticks
   ```
5. Repeat at 2-3 different velocities to verify K is consistent.

**Done when**: You have K with <3% variation across test points. Update `VELOCITY_CONVERSION_FACTOR` and `VELOCITY_TO_TICKS`.

---

### Step 4: Tune Pass-Through Point Constants

**What you need**: Validated LUT (Step 2), calibrated K (Step 3). Still `SHOOT_WHILE_MOVING_ENABLED = false`.

If using the **full physics mode** (replacing LUT), tune these. If using the **hybrid mode** (LUT + physics compensation for movement), skip to Step 5 — the LUT handles stationary shots.

Starting values:
```java
SCORE_HEIGHT = 24.53          // Goal lip - launcher height
SCORE_ANGLE = toRadians(-30)  // Ball descending at -30 deg
PASS_THROUGH_POINT_RADIUS = 5 // inches
```

**Procedure**:
1. Shoot from 80 inches (mid-range). Observe miss pattern.
2. Adjust `SCORE_HEIGHT` first:
   - Shots high: decrease by 1-2 inches
   - Shots low: increase by 1-2 inches
   - Re-test. Repeat until 80-inch shots are consistent.
3. Test at 50 inches and 130 inches without changing `SCORE_HEIGHT`.
4. Adjust `SCORE_ANGLE`:
   - Close range good, far range high: make SCORE_ANGLE more negative (e.g., -35 deg)
   - Close range good, far range low: make SCORE_ANGLE less negative (e.g., -25 deg)
   - Re-test at both distances. Repeat.
5. Adjust `PASS_THROUGH_POINT_RADIUS`:
   - If shots arc too high and drop in steeply: increase radius (brings target closer)
   - If shots come in too flat and skip off the lip: decrease radius (pushes target toward goal)
   - This has a subtle effect. Only adjust if Steps 2-4 don't fully solve the issue.
6. Final validation: test at 40, 80, 120, 150 inches. All should score 4/5+.

**Done when**: Physics-calculated shots match or exceed LUT accuracy at all distances.

---

### Step 5: Enable and Tune Shoot-While-Moving

**Prerequisites**: Steps 1-3 complete. LUT validated. K calibrated.

1. Set `SHOOT_WHILE_MOVING_ENABLED = true`.
2. **First test: Stationary regression check**. The robot should still shoot accurately when not moving. If not, there's a code bug — the system should fall back to LUT/stationary behavior below `LEAD_VELOCITY_DEADBAND`.

#### Phase A: Radial Compensation (Toward/Away from Goal)

3. Position robot ~100 inches from goal.
4. Drive the robot **straight toward the goal** at a slow, steady speed (~15 in/s). Shoot. Observe:
   - **Shots go long (over the goal)**: Radial compensation is ADDING velocity when it should SUBTRACT (or vice versa). Check the sign of `radialVelocity`. When moving toward the goal, `radialVelocity` should be positive, and the ball should need LESS flywheel power.
   - **Shots go short (under the goal)**: Radial compensation is subtracting too much. Check the sign convention.
   - **Shots correct**: Move to next test.
5. Drive **straight away from the goal** at ~15 in/s. Shoot. Verify the opposite compensation applies.
6. Increase speed to ~30 in/s. Re-test both directions. If shots degrade at higher speed:
   - May need to increase `LEAD_VELOCITY_DEADBAND` (filters out noise but reduces low-speed compensation)
   - May need to add EMA smoothing to velocity components
   - The compensation may be nonlinear at high speeds due to air resistance on the wiffle ball

#### Phase B: Tangential Compensation (Strafing)

7. Position robot ~100 inches from goal.
8. Strafe **left at ~15 in/s** while shooting. Observe:
   - **Shots miss to the left**: Turret isn't leading enough. Increase turret compensation. Check sign of `turretVelCompOffset`.
   - **Shots miss to the right**: Turret is over-leading. Reduce compensation or check sign.
   - **Shots correct left/right but wrong height**: The tangential velocity component is being miscalculated. The hood angle adjustment should be small for pure strafing.
9. Strafe **right at ~15 in/s**. Verify compensation is symmetric.
10. Increase strafe speed. Re-test.

#### Phase C: Combined Compensation (Diagonal Movement)

11. Drive at **~45 degrees to the goal** at ~20 in/s. This tests both radial and tangential simultaneously.
12. If individual phases work but combined fails:
    - Check that radial and tangential components are being computed independently and combined correctly
    - Verify the Pythagorean combination: `newHorizontalVelocity = sqrt(radial^2 + tangential^2)`

#### Phase D: Full-Speed Validation

13. Simulate match conditions: drive at realistic speeds while shooting from various distances and angles.
14. If accuracy degrades at high speed:
    - Add stronger velocity filtering (increase `SMOOTHING_ALPHA` or add a dedicated velocity EMA)
    - Consider reducing max driving speed during shots
    - The physics model assumes no air resistance; at high compensation values, errors compound

**Done when**: Shots score at 3/5+ rate while moving at match speeds across multiple field positions.

---

### Step 6: Alliance-Specific Calibration

1. Run the full test suite for **both Red and Blue** alliance.
2. The turret tracking offsets (`BLUE_TURRET_TRACKING_OFFSET`, `RED_TURRET_TRACKING_OFFSET`) compensate for asymmetry in the turret/Limelight mounting. Adjust independently for each alliance until both track accurately.
3. The physics model itself should NOT need per-alliance tuning (the goal positions in FieldMap handle the difference).

---

## 2. Pre-Match Verification Checklist

Run through this before every match. Takes ~2 minutes.

- [ ] **Battery voltage > 12.5V** (below 12V, flywheel can't reach max velocity)
- [ ] **Flywheel spin-up test**: Spin up to 2200 ticks/s, verify it reaches target within 2 seconds and holds steady (< 20 ticks variation)
- [ ] **Hood range test**: Command hood to 30 deg, then 63 deg. Listen for the servo moving. Verify no grinding or stalling.
- [ ] **Turret sweep test**: Command turret from -70 deg to +55 deg. Verify smooth motion, no skipping.
- [ ] **Alliance color set correctly** in robot configuration (affects goal position AND turret offset)
- [ ] **Odometry check**: Push robot ~12 inches in each direction, verify Pinpoint reports correct displacement on dashboard
- [ ] **Stationary shot test**: Shoot 2 balls from a known position. Both should score.
- [ ] **SHOOT_WHILE_MOVING_ENABLED** is set to your intended value for this match

---

## 3. Troubleshooting: Stationary Shooting

### 3.1 All Shots High (Ball Goes Over the Goal) — All Distances

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SCORE_HEIGHT` too high | Check: `38.75 - LAUNCHER_HEIGHT`. Measure launcher height. | Decrease `SCORE_HEIGHT` by 1-2 inches |
| 2 | Hood servo calibration off — physical angle is steeper than code thinks | Command hood to known angle, measure with protractor | Recalibrate `HOOD_SERVO_AT_MIN_ANGLE` / `HOOD_SERVO_AT_MAX_ANGLE` |
| 3 | `VELOCITY_CONVERSION_FACTOR` too low (physics mode) — calculated velocity is too low, so physics requests too many ticks | Recalculate K from LUT method | Increase K |
| 4 | LUT velocities too high (LUT mode) | Reduce all LUT velocity entries by 20-40 ticks | Adjust LUT entries |

### 3.2 All Shots Low (Ball Hits Below Goal Lip) — All Distances

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SCORE_HEIGHT` too low | Remeasure launcher height | Increase `SCORE_HEIGHT` by 1-2 inches |
| 2 | Flywheel wheels worn — less grip = less ball speed | Inspect rubber surface for wear/glazing | Replace Rhino wheels or scuff surface with sandpaper |
| 3 | LUT needs velocity increase | User noted +20 ticks globally needed | Add +20 to all LUT velocity entries |
| 4 | Battery voltage low — flywheel can't reach target | Check dashboard: `currentVelocity` vs `targetVelocity` | Charge/replace battery. Enable `VOLTAGE_COMPENSATION_ENABLED` |
| 5 | Ball is heavier than expected (absorbed moisture, different batch) | Weigh the ball | Use a fresh, dry ball. If persistent, increase LUT velocities |

### 3.3 All Shots Miss Left — All Distances

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Turret tracking offset wrong for current alliance | Switch alliance, check if it reverses | Adjust `BLUE_TURRET_TRACKING_OFFSET` or `RED_TURRET_TRACKING_OFFSET` (increase to shift aim right) |
| 2 | Turret servo center position miscalibrated | Command turret to 0 deg, check if physically centered | Adjust `SERVO_CENTER_POSITION` |
| 3 | Odometry heading drift | Drive robot in a square, check if heading returns to original | Recalibrate Pinpoint yaw offset |
| 4 | Turret offset constants wrong (`TURRET_OFFSET_X/Y`) | Remeasure turret pivot position relative to robot center | Update `TURRET_OFFSET_X`, `TURRET_OFFSET_Y` |

### 3.4 All Shots Miss Right — All Distances

Same as 3.3, but reverse the direction of fixes (decrease offsets instead of increase, etc.)

### 3.5 Close Range Accurate, Far Range High

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SCORE_ANGLE` too shallow (not negative enough) | Shots arc too much at distance | Make `SCORE_ANGLE` more negative (e.g., -30 deg to -35 deg) |
| 2 | Far-range LUT hood angles too steep | Compare LUT hood angle to what physics predicts | Increase hood angles for far LUT entries (more horizontal = flatter shot) |
| 3 | Far-range LUT velocities too high | Decrease LUT velocity entries for >120 inch distances | Reduce by 20-40 ticks |

### 3.6 Close Range Accurate, Far Range Low

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SCORE_ANGLE` too steep (too negative) | Shots don't arc enough at distance | Make `SCORE_ANGLE` less negative (e.g., -30 deg to -25 deg) |
| 2 | Far-range LUT velocities too low | Ball visibly losing speed at apex | Increase LUT velocity entries for >120 inches |
| 3 | Air resistance effect — wiffle ball decelerates significantly at long range | Physics model over-predicts range | Add a drag compensation factor to far-range LUT entries, or increase velocities empirically |

### 3.7 Far Range Accurate, Close Range High

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Close-range LUT hood angles too steep (too far from vertical) | Balls arc way over the goal from close | Decrease hood angles for <60 inch LUT entries |
| 2 | Close-range LUT velocities too high | Decrease velocity for <60 inch entries |
| 3 | `PASS_THROUGH_POINT_RADIUS` too large (physics mode) | Target point is too close to robot | Decrease `PASS_THROUGH_POINT_RADIUS` by 2-3 inches |

### 3.8 Far Range Accurate, Close Range Low

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Close-range LUT velocities too low | Increase velocity for <60 inch entries |
| 2 | Ball contact geometry — at steep hood angles (<45 deg), ball separates early from flywheel, getting less speed | This is a known property of the 5" ball + 72mm flywheel | Increase close-range LUT velocities more aggressively. Consider using `K_BELOW_45` if in physics mode. |

### 3.9 Shots Inconsistent (Some Hit, Some Miss, No Pattern)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Flywheel velocity not stabilized before shot | Check: is `isAtTargetVelocity()` true when shot fires? | Add/increase wait in shooting sequence. Check `VELOCITY_TOLERANCE` isn't too loose. |
| 2 | Ball condition varies — some balls deformed, wet, or different weight | Try with all-new identical balls | Use consistent balls. Inspect before loading. |
| 3 | Hood servo backlash | Command hood to same angle from above and below, see if it lands differently | Add anti-backlash: always approach from the same direction, or replace with a higher-quality servo |
| 4 | Flywheel motor gearbox play | Spin flywheel by hand with motors off, feel for slop | Tighten set screws. If gearbox is worn, rebuild or replace. |
| 5 | Ball not seated consistently in spindexer slot 1 | Watch ball position before each shot | Check spindexer rotation accuracy. Verify flipper timing. |
| 6 | Turret servo jitter from noisy angle calculation | Watch turret on dashboard — does it oscillate? | Increase `SMOOTHING_ALPHA` toward 1.0 (less smoothing) or decrease if oscillating |

### 3.10 Shots Hit the Lip/Rim of the Goal

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SCORE_HEIGHT` is exactly right but no safety margin | Ball trajectory barely clears the lip | Increase `SCORE_HEIGHT` by 1-2 inches to add clearance |
| 2 | From specific distances, the trajectory grazes the lip | Map which distances have this issue | Tweak LUT entries for those distances (+10-20 ticks velocity, +1-2 deg hood angle) |

### 3.11 Shots Hit the Backboard Hard and Bounce Out

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Shots too flat — ball enters nearly horizontally | Watch ball trajectory | Steepen the entry: make `SCORE_ANGLE` more negative, or decrease hood angle (steeper launch) |
| 2 | Too much velocity — ball bounces off backboard | Reduce LUT velocities slightly, or reduce `PASS_THROUGH_POINT_RADIUS` to aim the pass-through point deeper into the goal |

---

## 4. Troubleshooting: Shoot-While-Moving

All issues in this section assume stationary shooting works correctly. If it doesn't, fix Section 3 first.

### 4.1 Shots Drift in the Direction of Robot Movement

**Example**: Robot strafes right, shots miss to the right.

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Tangential turret compensation has wrong sign | Reverse strafe direction, see if miss reverses | Flip the sign: `Math.atan2(-tangentialVelocity, adjustedRadialVelocity)` |
| 2 | Tangential compensation is not being applied | Check dashboard: is `turretVelCompOffset` non-zero while moving? | Verify the turret is reading the offset from the shooter. Check `SHOOT_WHILE_MOVING_ENABLED`. |
| 3 | Robot velocity is in wrong frame (robot-relative vs field-relative) | Drive north, check that `fieldVelY` is positive | Review the rotation transform: `fieldVelX = velX*cos(h) - velY*sin(h)` |

### 4.2 Shots Drift Opposite to Robot Movement

**Example**: Robot strafes right, shots miss to the left.

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Tangential compensation is OVER-correcting (double-applied) | Reduce robot speed, check if miss magnitude decreases proportionally | The offset is being applied twice (once in computation, once by existing lead compensation). Ensure old `updateLeadCompensation()` is fully replaced, not layered on top. |
| 2 | Turret is already leading via the old future-pose method AND the new tangential offset | Check if both `getDegreesToGoalLeadAdjusted` (old) and `turretVelCompOffset` (new) are active | Disable old lead compensation when new compensation is active |

### 4.3 Shots Go Long When Driving Toward Goal

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Radial compensation sign wrong — adding velocity when should subtract | Dashboard: check `radialVelocity` is positive when driving toward goal | The ball inherits the robot's forward velocity. The flywheel should spin SLOWER: `adjustedRadialVelocity = (x/t) + radialVelocity` where radialVelocity is positive → higher total velocity → physics should calculate LESS flywheel ticks. If flywheel ticks go UP, the conversion is inverted. |
| 2 | `VELOCITY_CONVERSION_FACTOR` inaccurate — compensation math produces wrong absolute values | Recalibrate K (Step 3) | Even small K errors amplify in the compensation loop |

### 4.4 Shots Go Short When Driving Toward Goal

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Compensation subtracting too much — flywheel velocity drops too low | Check: compensated ticks vs stationary ticks at same distance | The ball doesn't fully inherit robot velocity (partially absorbed by the flywheel mechanism). Add a `RADIAL_COMPENSATION_SCALE` factor (0.0-1.0, start at 0.7) to scale down the radial component: `radialVelocity *= RADIAL_COMPENSATION_SCALE` |
| 2 | Pinpoint reports velocity higher than actual | Compare Pinpoint velocity to expected speed from motor commands | Recalibrate Pinpoint wheel radii or encoder resolution |

### 4.5 Shots Go Long When Driving Away from Goal

Same root causes as 4.4 but reversed. The compensation isn't adding enough velocity to counteract the robot moving away.

### 4.6 Shots Go Short When Driving Away from Goal

Same root causes as 4.3 but reversed.

### 4.7 Turret Doesn't Lead Enough (Shots Consistently Behind)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Time-in-air estimate too low — ball is in the air longer than predicted | Calculate actual time: `x / (v0 * cos(launchAngle))` at several distances. Compare to what the code uses. | If using constant `TIME_IN_AIR = 0.675`, switch to the physics-based calculation which varies by distance |
| 2 | Velocity filtering too aggressive — by the time the system reacts, the robot has moved past where it predicted | Check `LEAD_VELOCITY_DEADBAND`. If > 5 in/s, the system ignores most movement | Decrease `LEAD_VELOCITY_DEADBAND` (try 2.0) |
| 3 | EMA smoothing on turret is lagging | Turret angle on dashboard lags behind the raw target | Increase `SMOOTHING_ALPHA` (closer to 1.0 = less lag). Trade-off: more responsive but more jittery |

### 4.8 Turret Leads Too Much (Shots Consistently Ahead)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Time-in-air estimate too high | See 4.7 #1 | Switch to physics-based time calculation |
| 2 | Old and new lead compensation both active | Check code: is the old `futurePose` prediction running alongside the new tangential offset? | Ensure only ONE compensation method is active |
| 3 | Pinpoint velocity spikes (noise) | Log raw velocity values | Add EMA filter to velocity components before decomposition. Start with alpha = 0.3 |

### 4.9 Compensation Works at Low Speed, Fails at High Speed

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | No-air-resistance assumption breaks down at large compensations | The wiffle ball experiences significant drag; large velocity deltas amplify the error | Add a `MAX_COMPENSATION_DELTA` cap. Limit how much the compensated velocity can differ from the stationary baseline (e.g., max +/- 300 ticks) |
| 2 | Servo response time — hood and turret can't keep up with rapid changes | Watch servos during fast driving — are they lagging? | Add a speed-dependent filter: at high speeds, smooth the outputs more aggressively |
| 3 | Velocity readings noisy at high speed | Plot velocity on dashboard while driving fast | Increase velocity filtering. Consider a moving average over 3-5 samples |

### 4.10 Shots Jittery While Moving (Oscillate Between Hit and Miss)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Velocity readings oscillating → hood/flywheel oscillate | Watch `fieldVelX`, `fieldVelY` on dashboard while driving | Increase `LEAD_VELOCITY_DEADBAND` to ignore micro-movements. Add EMA to velocity components. |
| 2 | `SMOOTHING_ALPHA` too high (too responsive) on turret | Turret visibly oscillating | Decrease `SMOOTHING_ALPHA` (e.g., from 0.35 to 0.20) |
| 3 | Hood servo can't track rapid angle changes | Hood angle on dashboard changes faster than servo can physically move | Add rate limiting to hood angle changes (max deg/sec) |

### 4.11 Compensation Activates When Robot Is Stationary

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `LEAD_VELOCITY_DEADBAND` too low — sensor noise triggers compensation | Check: robot is still, but `fieldVelX`/`fieldVelY` are non-zero | Increase `LEAD_VELOCITY_DEADBAND` to 3.0-5.0 in/s |
| 2 | Heading velocity noise triggers heading compensation | Check `cachedHeadingVel` while stationary | Increase `LEAD_HEADING_VELOCITY_DEADBAND` to 0.1 rad/s |

---

## 5. Troubleshooting: Flywheel

### 5.1 Flywheel Takes Too Long to Spin Up (> 2 Seconds)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | PID gains too conservative | Watch spin-up curve on dashboard | Increase `VELOCITY_kP` (try 0.003). Increase `kA` slightly. |
| 2 | Low battery voltage | Check voltage in telemetry | Charge battery. Enable `VOLTAGE_COMPENSATION_ENABLED`. |
| 3 | Mechanical resistance (belt tension, bearing friction) | Spin flywheel by hand — is it smooth? | Lubricate bearings, check belt tension, ensure no rubbing |
| 4 | One motor not working | Run each motor independently | Check wiring, motor port assignment, direction configuration |

### 5.2 Flywheel Velocity Oscillates Around Target

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | PID `kP` too high | Velocity overshoots then undershoots target | Decrease `VELOCITY_kP` by 25% |
| 2 | `kI` causing integral windup | `integralSum` grows large on dashboard | Decrease `INTEGRAL_MAX` or decrease `VELOCITY_kI` |
| 3 | Motor 1 encoder is dead (known issue) — reading only motor 2, but both motors may not match | Power mismatch between motors | This is a known issue. Ensure both motors have same wiring, gearing, and load. Consider reading motor 1 velocity if encoder is repaired. |

### 5.3 Velocity Drops During Rapid Fire (Multi-Ball Sequence)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Each ball absorbs flywheel energy — normal behavior | Velocity dips then recovers between shots | Increase delay between shots (`SHOOTER_RETRACT_DELAY`) to allow recovery. Pre-spin to higher velocity before multi-shot sequence. |
| 2 | Battery voltage sag under load | Check voltage during rapid fire | Enable `VOLTAGE_COMPENSATION_ENABLED`. Use a higher-capacity battery. |

### 5.4 Flywheel Never Reaches Target Velocity

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Target exceeds motor capability at current voltage | Is target > 2800 ticks/s with battery < 13V? | Reduce target velocity. Charge battery. |
| 2 | `kS` (static friction compensation) too low | Motor barely moves at low targets | Increase `kS` (try 0.05) |
| 3 | Mechanical issue — friction, misalignment | Motor gets hot quickly | Inspect flywheel assembly for rubbing, misalignment |
| 4 | Motor 2 encoder miscounting | Velocity reading seems wrong relative to motor sound | Check encoder wiring, try reading motor 1 if available |

---

## 6. Troubleshooting: Hood

### 6.1 Hood Hits Mechanical Limits Frequently

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Operating at extreme distances (very close or very far) | Check what distance triggers the limit | If physics mode calculates angles outside 30-63 deg, the shot is outside the launcher's effective range. Don't shoot from those distances. |
| 2 | Velocity compensation pushing hood angle outside range | Only happens while moving | Add a check: if compensated hood angle would be clamped, don't apply compensation (fall back to stationary shot) |

### 6.2 Hood Servo Jitters

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Distance calculation fluctuating → hood angle fluctuating | Watch `requiredHoodAngle` on dashboard — does it bounce? | The dirty flag (`SERVO_EPSILON = 0.001`) should filter this, but if the angle changes by more than epsilon each loop, it will write every time. Increase `SERVO_EPSILON` to 0.005. |
| 2 | Servo power issue (brownout) | Servo buzzes or jumps | Check servo power source. Add a servo power hub if needed. |

### 6.3 Hood Physical Angle Doesn't Match Commanded Angle

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Servo calibration drift | Measure physical angle at several commanded positions | Recalibrate `HOOD_SERVO_AT_MIN_ANGLE` and `HOOD_SERVO_AT_MAX_ANGLE`. The mapping is linear, so two points define the line. |
| 2 | Servo linkage slipping | Hood doesn't move when servo does | Tighten set screws, check linkage for wear |

---

## 7. Troubleshooting: Turret

### 7.1 Turret Oscillates / Jitters

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | `SMOOTHING_ALPHA` too high (too responsive to noise) | Raw target vs smoothed target on dashboard | Decrease `SMOOTHING_ALPHA` (try 0.20-0.25) |
| 2 | Odometry heading noise | Check heading stability while stationary | This is a Pinpoint calibration issue. Recalibrate heading offset. |
| 3 | Turret gear backlash | Turret wiggles physically even when servo is steady | Add deadband: `MIN_CHANGE_THRESHOLD` is already 0.5 deg. Increase to 1.0 if needed. |

### 7.2 Turret Hits Hard Stops During Match

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Robot is oriented such that the goal is behind the turret range | Check robot heading vs goal direction | This is a driving/strategy issue. The driver should avoid headings where the turret can't reach. Consider adding a dashboard warning when turret is near limits. |
| 2 | Hard stop values don't match physical limits | Command turret to limit values, verify it doesn't grind | Adjust `HARD_STOP_CW` / `HARD_STOP_CCW` to match actual physical range |

### 7.3 Turret Tracks Correctly for Red but Not Blue (or Vice Versa)

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Alliance-specific tracking offsets not calibrated | Swap alliance color, re-test | Calibrate `BLUE_TURRET_TRACKING_OFFSET` and `RED_TURRET_TRACKING_OFFSET` independently |
| 2 | Goal position constants wrong | Verify `RED_GOAL_X/Y` and `BLUE_GOAL_X/Y` match field layout | Remeasure or check game manual coordinates |

---

## 8. Troubleshooting: System-Level

### 8.1 First Shot After Spin-Up Misses, Subsequent Shots Hit

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Flywheel not fully stabilized — `isAtTargetVelocity()` triggers early | Log velocity at time of first shot | Tighten `VELOCITY_TOLERANCE` (from 20 to 10 ticks). Add a "hold at target for 100ms" requirement. |
| 2 | Hood servo hasn't finished moving to position | First shot fires before hood settles | Add a minimum delay between hood angle change and first shot |
| 3 | Thermal effect — flywheel wheels perform differently when cold vs warm | First shot of the match is always off | Pre-spin flywheel during init to warm up the rubber |

### 8.2 Accuracy Degrades Over the Course of a Match

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Battery voltage sag | Log voltage at start vs end of match | Enable `VOLTAGE_COMPENSATION_ENABLED`. Start match with fully charged battery (>13V). |
| 2 | Flywheel rubber heating up — grip changes | Touch wheel after match — is it hot? | This is normal for 30A rubber under sustained use. Re-tune LUT with "warm" wheels (after 30 seconds of spinning). |
| 3 | Odometry drift accumulating | Check pose accuracy at end of match | Use Limelight re-localization periodically |

### 8.3 Shots Work in Practice but Not in Competition

| # | Possible Cause | How to Verify | Fix |
|---|---------------|---------------|-----|
| 1 | Different ball condition (competition balls may be different wear/age) | Ask to see the balls, compare to practice balls | Always practice with balls in similar condition to competition. Bring backup balls. |
| 2 | Field tile friction differs — robot slides, odometry drifts | Check if robot stops where expected during auto | Recalibrate Pedro Pathing follower PIDs for the specific field |
| 3 | Different lighting affects Limelight — but this shouldn't affect the physics shooter | Verify Limelight detections are stable | This only matters if Limelight is used for re-localization, not directly for shot calculation |
| 4 | Competition stress → different driving patterns → different velocities | Watch match video — is the driver moving faster/more erratically? | Practice under match-like conditions. Consider reducing `MOVING_WHILE_SHOOTING_VELOCITY_THRESHOLD` to only compensate when actually moving significantly. |

---

## 9. Quick-Reference Cheat Sheet

Print this page and keep it in your pit.

### Shots Too High
1. Decrease `SCORE_HEIGHT` by 1-2"
2. OR increase hood angle entries in LUT (flatter shot)
3. OR decrease LUT velocity entries

### Shots Too Low
1. Increase `SCORE_HEIGHT` by 1-2"
2. OR decrease hood angle entries in LUT (steeper shot)
3. OR increase LUT velocity entries (+20 ticks is a known pending fix)

### Shots Miss Left
- Increase turret tracking offset for current alliance (+0.5 deg increments)

### Shots Miss Right
- Decrease turret tracking offset for current alliance (-0.5 deg increments)

### Close Good, Far Bad
- Adjust `SCORE_ANGLE`: more negative = fix high far shots, less negative = fix low far shots

### Inconsistent Shots
1. Check battery (>12.5V)
2. Check ball condition
3. Tighten `VELOCITY_TOLERANCE` to 10
4. Check flywheel for mechanical play

### Moving Shots Drift Sideways
- Check turret velocity compensation sign
- Ensure old lead compensation is disabled if new system is active

### Moving Shots Wrong Distance
- Check radial velocity sign convention
- Add `RADIAL_COMPENSATION_SCALE` (start at 0.7)

### Emergency: Nothing Works
1. Set `SHOOT_WHILE_MOVING_ENABLED = false`
2. Set `ShooterTuning.TUNING_MODE = false`
3. Verify LUT is loaded correctly (check `velocityLUT.get(100)` returns ~2180)
4. Re-run pre-match checklist

---

## 10. Constants Reference

All tunable constants and their locations, grouped by subsystem.

### ShooterConstants.java — Physics

| Constant | Default | Description | Adjustment Effect |
|----------|---------|-------------|-------------------|
| `SCORE_HEIGHT` | 24.53 | Vertical distance from launcher to target point (inches) | Increase = higher shots |
| `SCORE_ANGLE` | -30 deg | Ball descent angle at target point (degrees, negative) | More negative = steeper entry, helps far range |
| `PASS_THROUGH_POINT_RADIUS` | 5.0 | Horizontal offset of target point from goal (inches) | Increase = shallower entry |
| `VELOCITY_CONVERSION_FACTOR` | 0.101 | Ticks/sec to in/s conversion | Increase = physics requests fewer ticks (lower power) |
| `VELOCITY_TO_TICKS` | ~9.9 | Inverse of above | Auto-calculated |
| `GRAVITY` | 386.088 | Gravitational acceleration (in/s^2) | Do not change |

### ShooterConstants.java — Control

| Constant | Default | Description | Adjustment Effect |
|----------|---------|-------------|-------------------|
| `SHOOT_WHILE_MOVING_ENABLED` | false | Master enable for velocity compensation | Set true after calibration |
| `LEAD_VELOCITY_DEADBAND` | 3.0 | Min speed (in/s) for compensation to activate | Increase = less jitter, less low-speed compensation |
| `LEAD_HEADING_VELOCITY_DEADBAND` | 0.05 | Min heading rate (rad/s) for heading compensation | Increase = less heading jitter |
| `TIME_IN_AIR` | 0.675 | Constant flight time (seconds) — replaced by physics calc when enabled | Only used as fallback |
| `VELOCITY_TOLERANCE` | 20.0 | Ticks/sec tolerance for "ready" | Decrease for accuracy, increase for faster shooting |
| `MOVING_WHILE_SHOOTING_VELOCITY_THRESHOLD` | 2.0 | Min speed for TeleOp lead compensation | Increase if jittery at low speeds |

### ShooterConstants.java — PIDKSV

| Constant | Default | Description | Adjustment Effect |
|----------|---------|-------------|-------------------|
| `kS` | 0.03 | Static friction compensation | Increase if motor doesn't start at low targets |
| `kV` | 0.00029 | Velocity feedforward gain | Primary spin-up driver. Increase = more power per tick/s |
| `kA` | 0.00001 | Acceleration feedforward gain | Increase for faster response, but may overshoot |
| `VELOCITY_kP` | 0.002 | Proportional gain | Increase for tighter tracking, decrease if oscillating |
| `VELOCITY_kI` | 0.00001 | Integral gain | Increase to eliminate steady-state error. Watch for windup |
| `VELOCITY_kD` | 0.0 | Derivative gain (on measurement) | Usually leave at 0 for flywheels |
| `INTEGRAL_MAX` | 0.3 | Max integral accumulation | Decrease if integral windup is a problem |

### TurretConstants.java

| Constant | Default | Description | Adjustment Effect |
|----------|---------|-------------|-------------------|
| `SMOOTHING_ALPHA` | 0.35 | EMA smoothing (0=frozen, 1=raw) | Decrease = smoother but laggier |
| `HARD_STOP_CW` | 55.0 | Max clockwise rotation (degrees) | Match to physical limit |
| `HARD_STOP_CCW` | -70.0 | Max counter-clockwise rotation (degrees) | Match to physical limit |
| `BLUE_TURRET_TRACKING_OFFSET` | 0.0 | Blue alliance aim correction (degrees) | Increase = aim more right |
| `RED_TURRET_TRACKING_OFFSET` | 1.5 | Red alliance aim correction (degrees) | Increase = aim more right |
| `TURRET_OFFSET_X` | 4.0 | Turret pivot X offset from robot center (inches, right) | Must match physical measurement |
| `TURRET_OFFSET_Y` | 1.0 | Turret pivot Y offset from robot center (inches, forward) | Must match physical measurement |

### LUT Data (ShooterConstants.java)

When adjusting the LUT, change entries in pairs (velocity AND hood angle) at each distance. After changing, re-validate from at least 3 distances.

| Distance | Velocity (ticks/s) | Hood Angle (deg) | Launch Angle (deg from horiz) |
|----------|--------------------|-----------------|-----------------------------|
| 40 | 1660 | 32 | 58 |
| 50 | 1760 | 37 | 53 |
| 60 | 1880 | 41 | 49 |
| 70 | 2000 | 43 | 47 |
| 80 | 2100 | 45 | 45 |
| 90 | 2140 | 46 | 44 |
| 100 | 2180 | 47 | 43 |
| 114 | 2320 | 49.5 | 40.5 |
| 125 | 2540 | 51.5 | 38.5 |
| 132 | 2620 | 52 | 38 |
| 140 | 2640 | 53 | 37 |
| 146 | 2680 | 53.25 | 36.75 |
| 151 | 2760 | 55 | 35 |
| 158 | 2860 | 57 | 33 |

**Pending fix**: All velocity entries need +20 ticks/sec (not yet applied in current codebase).

### Ball Contact Note
At 45 deg hood angle (45 deg launch), the 5-inch ball loses contact with the 72mm flywheel at the exit. Hood angles below 45 deg (steeper launch) have slightly less velocity transfer. If per-range accuracy matters, consider a piecewise `VELOCITY_CONVERSION_FACTOR`:
- Hood angle 30-45 deg: K = 0.100
- Hood angle 45-63 deg: K = 0.103
