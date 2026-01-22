# Shooter & Turret Velocity Compensation Tuning Guide

This guide explains how to tune the shoot-while-moving velocity compensation system.

## Prerequisites
1. Static shooting must already be working and accurate
2. Your velocity/hood LUTs should be calibrated for stationary shots
3. Flywheel feedforward should be tuned (see ShooterFeedforwardTuning.md)

---

## Quick Start

### Step 1: Verify Static Baseline
Before tuning velocity compensation, confirm static shots work:

1. Set `VELOCITY_COMPENSATION_ENABLED = false` in ShooterConstants
2. Shoot from 3-4 distances while stationary
3. All shots should hit consistently
4. If not, fix your static LUTs first

### Step 2: Enable Compensation
1. Set `VELOCITY_COMPENSATION_ENABLED = true`
2. Start with conservative coefficients:
   - `RADIAL_VELOCITY_COEFFICIENT = 0.5`
   - `TANGENTIAL_VELOCITY_COEFFICIENT = 0.5`
3. Verify telemetry shows `Adjusted Velocity ≈ Base Velocity` when stationary

---

## Tuning Parameters

### VELOCITY_FILTER_ALPHA (0.0 - 1.0)
**What it does:** Smooths noisy velocity readings using exponential moving average.
- Higher (0.5-0.8) = Less smoothing, faster response, more noise
- Lower (0.1-0.3) = More smoothing, slower response, less noise

**How to tune:**
1. Watch telemetry while robot is stationary
2. If `Robot Vel X/Y` fluctuates > ±2 in/s, decrease alpha
3. If values lag behind actual motion, increase alpha
4. Default: 0.3

### BALL_EXIT_DELAY (seconds)
**What it does:** Predicts where robot will be when ball exits shooter.

**How to tune:**
1. Record slow-motion video of a shot
2. Count frames from trigger press to ball leaving shooter
3. Convert to seconds: `delay = frames / fps`
4. Typical value: 0.060 - 0.100 seconds

**Testing:**
1. Set to 0 (no prediction) - shots should lag behind at high speeds
2. Increase until shots lead correctly
3. If overshooting, reduce slightly

### RADIAL_VELOCITY_COEFFICIENT
**What it does:** Controls velocity adjustment when moving toward/away from goal.

**Physics:**
- Moving toward goal -> ball already has that velocity -> reduce flywheel speed
- Moving away from goal -> ball loses that velocity -> increase flywheel speed

**How to tune:**
1. Drive TOWARD goal at ~2 ft/s while shooting
2. If shots go LONG (too much power), increase coefficient
3. If shots go SHORT (not enough power), decrease coefficient
4. Drive AWAY from goal at ~2 ft/s
5. Same tuning: long = increase, short = decrease

**Start value:** 1.0
**Typical range:** 0.7 - 1.3

### TANGENTIAL_VELOCITY_COEFFICIENT
**What it does:** Controls lead angle when moving perpendicular to goal.

**Physics:**
- Strafing right -> ball drifts right -> aim left to compensate
- Strafing left -> ball drifts left -> aim right to compensate

**How to tune:**
1. Strafe RIGHT at ~2 ft/s while shooting
2. If shots miss RIGHT (under-leading), increase coefficient
3. If shots miss LEFT (over-leading), decrease coefficient
4. Strafe LEFT - same tuning principle

**Start value:** 1.0
**Typical range:** 0.7 - 1.3

### TICKS_TO_INCHES_PER_SEC
**What it does:** Converts flywheel ticks/sec to ball speed in inches/sec.
Used for lead angle calculation.

**How to calculate:**
1. Measure ball exit speed using slow-motion video
2. Record flywheel velocity (ticks/sec) during that shot
3. `TICKS_TO_INCHES_PER_SEC = ball_speed_inches / flywheel_ticks`

**Example:**
- Ball travels 10 feet (120 inches) in 1.5 seconds = 80 in/s
- Flywheel was at 2000 ticks/sec
- Coefficient = 80 / 2000 = 0.04

**Start value:** 0.05
**Typical range:** 0.03 - 0.07

---

## Safety Parameters

### MAX_VELOCITY_ADJUSTMENT
Maximum allowed velocity change (ticks/sec). If exceeded, `isSafeToShoot()` returns false.

**Purpose:** Prevents crazy velocity values when math goes wrong or robot moves very fast.

**How to set:**
1. Note your flywheel's typical operating range (e.g., 1800-2600 ticks/sec)
2. Set to ~15-20% of mid-range: `MAX = 0.15 * 2200 = 330`
3. Round up slightly for margin: 400

### MIN_SAFE_VELOCITY / MAX_SAFE_VELOCITY
Absolute limits for adjusted velocity.

**How to set:**
- MIN: Lowest velocity that still shoots reliably (test: ~1500)
- MAX: Highest velocity motors can sustain without issues (~3200)

### MAX_LEAD_ANGLE
Maximum turret lead angle (degrees) before rejecting shot.

**How to set:**
1. Check turret physical limits (HARD_STOP_CW, HARD_STOP_CCW)
2. Set MAX_LEAD_ANGLE < min(|CW|, |CCW|) - base_angle_margin
3. Typical: 10-15 degrees

---

## Troubleshooting

### Shots consistently miss in one direction when moving
- **Missing forward:** Decrease RADIAL_VELOCITY_COEFFICIENT
- **Missing backward:** Increase RADIAL_VELOCITY_COEFFICIENT
- **Missing left:** Increase TANGENTIAL_VELOCITY_COEFFICIENT (when strafing right)
- **Missing right:** Decrease TANGENTIAL_VELOCITY_COEFFICIENT

### Shots accurate when slow, miss when fast
- Increase BALL_EXIT_DELAY (more prediction)
- Or decrease speed threshold for shooting

### Telemetry velocity values jump around
- Decrease VELOCITY_FILTER_ALPHA (more smoothing)
- Check pinpoint connection and mounting

### "Safe to Shoot: false" too often
- Increase safety limits gradually
- Check if robot is moving too fast for reliable compensation
- Consider adding speed limit warning in telemetry

### Hood angle seems wrong when moving
- The hood adjustment uses effective distance: `distance * (baseVel / adjustedVel)`
- If velocities are close, hood change will be minimal
- Large velocity changes = larger hood adjustment

---

## Recommended Tuning Order

1. **VELOCITY_FILTER_ALPHA** - Get clean velocity readings first
2. **BALL_EXIT_DELAY** - Measure accurately, then fine-tune
3. **RADIAL_VELOCITY_COEFFICIENT** - Drive toward/away from goal
4. **TANGENTIAL_VELOCITY_COEFFICIENT** - Strafe perpendicular to goal
5. **TICKS_TO_INCHES_PER_SEC** - Fine-tune lead angle accuracy
6. **Safety limits** - Adjust based on observed behavior

---

## Telemetry Reference

Watch these values while tuning:

| Telemetry | What to look for |
|-----------|------------------|
| Robot Vel X/Y | Should be stable, ~0 when stationary |
| Base Velocity | Your LUT output (shouldn't change when moving) |
| Adjusted Velocity | Should decrease toward goal, increase away |
| Lead Angle | Should be + when strafing right, - when left |
| Adjusted Hood | Small changes unless velocity changes a lot |
| Safe to Shoot | Should be true at reasonable speeds |

---

## Competition Tips

1. **Practice matches:** Run with compensation ENABLED to tune
2. **If issues occur:** Quick fix is `VELOCITY_COMPENSATION_ENABLED = false`
3. **Conservative approach:** Use lower coefficients (0.7) for reliability
4. **Speed limits:** Consider blocking shots above certain robot speeds

Remember: A slightly inaccurate shot that hits is better than a perfectly calculated shot that misses!

---

## Technical Reference

### Velocity Decomposition
```
Target
   X
   |\
   | \  tangential (perpendicular)
   |  \
   |   \  robot velocity vector
   |    \
   |_____\
   radial (toward/away)

   Robot
```

### Compensation Formulas
```java
// Radial affects exit velocity
vRadial = robotSpeed * cos(angleToTarget - velocityAngle)
adjustedVelocity = baseVelocity - vRadial * RADIAL_COEFFICIENT

// Tangential affects turret lead
vTangential = robotSpeed * sin(angleToTarget - velocityAngle)
leadAngle = atan2(vTangential * TANGENTIAL_COEFFICIENT, ballExitSpeed)

// Hood adjusts for effective distance change
effectiveDistance = distance * (baseVelocity / adjustedVelocity)
adjustedHoodAngle = hoodLUT.get(effectiveDistance)
```

### Position Prediction
```java
predictedX = currentX + velocityX * BALL_EXIT_DELAY
predictedY = currentY + velocityY * BALL_EXIT_DELAY
// All calculations use predicted position, not current
```
