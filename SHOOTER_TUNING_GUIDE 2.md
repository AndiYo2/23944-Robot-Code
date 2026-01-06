# Shooter PIDF Tuning Guide

## What Was Fixed

### Problem
Both shooter motors were running at full power (1.0) constantly instead of maintaining target velocity properly.

### Root Cause
1. **Motor 1 wasn't using velocity control** - It was set to `RUN_WITHOUT_ENCODER` and just mirrored Motor 2's fluctuating power
2. **No integral gain (I = 0)** - Without I term, the motors couldn't fight steady-state loads or recover from shooting disturbances
3. **Possible low F gain** - If feedforward is too low, the P term saturates trying to compensate

### The Fix
Both motors now use:
- ✅ `RUN_USING_ENCODER` mode
- ✅ PIDF velocity control with `setVelocity()`
- ✅ Same PIDF coefficients applied to both motors
- ✅ Continuous velocity control in `periodic()` to maintain speed

---

## Understanding PIDF for Flywheels

### What Each Gain Does

**F (Feedforward)** - Primary gain for velocity control
- Provides baseline power based on target velocity
- **Most important for flywheels!**
- Formula: `power ≈ (F × targetVelocity) / scaleFactor`
- If F is too low → P saturates → full power
- If F is too high → overshoots target velocity

**P (Proportional)** - Error correction
- Fixes velocity errors: `P × (targetVelocity - actualVelocity)`
- If too high → oscillations and saturation
- If too low → slow response, never reaches target

**I (Integral)** - Fights steady-state loads
- Accumulates error over time to eliminate steady-state offset
- **Critical for maintaining velocity under load (shooting balls)**
- **Critical for recovering velocity after shooting**
- If too high → oscillations and overshoot
- If too low → can't maintain velocity under load

**D (Derivative)** - Dampens oscillations
- Usually not needed for flywheels
- Can help reduce overshoot
- Keep at 0 unless you have oscillation problems

---

## Tuning Process

### Step 1: Use the Tuning TeleOp

1. Run `ShooterPIDFTuningTeleOp` from the driver station
2. This allows you to adjust P and F gains in real-time
3. Watch telemetry for:
   - **Target Velocity** vs **Current Velocity**
   - **Error** (should be small, <50 ticks/sec)
   - **Motor Power** (should be 60-90%, NOT 1.0 constantly)

### Step 2: Tune F (Feedforward) First

**Goal:** Get motors to ~90% of target velocity with F alone (P=0)

1. Set P = 0 temporarily
2. Start with F = 6.4 (reference value from tuning TeleOp)
3. Observe motor power and velocity:
   - If velocity is too low AND power < 1.0 → Increase F
   - If velocity overshoots → Decrease F
   - If power = 1.0 constantly → Increase F significantly

**What to look for:**
- At target velocity 2200 ticks/sec:
  - Good: Power ~0.7-0.9, velocity ~2100-2200
  - Bad: Power = 1.0, velocity < 2000
  - Bad: Power ~0.5, velocity < 1800

**Typical F values for FTC motors:**
- Light flywheel: F = 10-15
- Heavy flywheel: F = 6-10
- Your current F = 7.4 (may be too low if motors saturate)

### Step 3: Tune P (Proportional)

**Goal:** Eliminate remaining velocity error quickly without oscillation

1. Start with P = 5.0 (current value)
2. Shoot a ball and observe recovery:
   - Too slow recovery → Increase P
   - Oscillates after shooting → Decrease P
   - Powers to 1.0 briefly then settles → Good!

**What to look for:**
- Error should be < 50 ticks/sec at steady state
- Recovery after shot should take < 0.5 seconds
- No oscillations (velocity bouncing up/down)

### Step 4: Add I (Integral) If Needed

**Goal:** Eliminate steady-state error and improve recovery

1. Start with I = 0.0 (current value)
2. If you have steady-state error (e.g., always 100 ticks/sec low):
   - Add small I = 0.1
   - Gradually increase to 0.3-0.5 if needed
3. If motors slow down during continuous shooting:
   - This means load is increasing
   - Increase I to fight the load

**Warning:** Too much I causes oscillations and overshoot!

### Step 5: Test Under Load

1. **Idle test:** Motors should maintain velocity with minimal power fluctuation
2. **Single shot test:** Shoot one ball, watch recovery time
3. **Rapid fire test:** Shoot 3 balls quickly, velocity should stay consistent
4. **Distance test:** Test at different distances (2100, 2200, 2600 velocities)

---

## Recommended Starting Values

Based on the ShooterPIDFTuningTeleOp reference:

```java
SHOOTER_F = 6.4  // Reference from tuning TeleOp
SHOOTER_P = 5.0  // Good starting point
SHOOTER_I = 0.0  // Start at 0, increase if needed
SHOOTER_D = 0.0  // Not needed for flywheels
```

**If motors are saturating (running at 1.0 constantly):**
```java
SHOOTER_F = 10.0  // Increase significantly
SHOOTER_P = 3.0   // Reduce to prevent saturation
SHOOTER_I = 0.2   // Add small I for load fighting
SHOOTER_D = 0.0   // Keep at 0
```

---

## Using Built-In Tuning Mode

Your TeleOp already has a built-in PIDF tuning mode!

### How to Use It:

1. **Enter tuning mode:** Press `Back` button on gamepad
   - You'll feel a short rumble
2. **Switch to shooter tuning:** Press `Y` button
   - Telemetry will show "SHOOTER PIDF TUNING MODE ACTIVE"
3. **Adjust parameters:**
   - `D-Pad Left/Right`: Select parameter (P, I, D, or F)
   - `D-Pad Up/Down`: Increase/decrease selected parameter
   - `B Button`: Change step size (10, 1, 0.1, 0.01, 0.001, 0.0001)
   - `X Button`: Trigger shooter flipper to test
4. **Monitor telemetry:**
   - Goal Velocity vs Motor Velocity
   - Velocity Error (should be near 0)
   - Motor Power (should NOT be 1.0 constantly)
5. **Exit tuning mode:** Press `Back` button again
   - Long rumble on exit
   - **Write down your final values!**

### Tuning Tips:

- Start with large step size (1.0) for coarse adjustments
- Switch to small step size (0.01 or 0.001) for fine tuning
- Test by triggering shots (X button) and watching recovery
- Watch for power saturation (power = 1.0 = BAD)
- Aim for error < 50 ticks/sec and power 0.6-0.9

---

## Expected Behavior After Tuning

### Good Signs ✅
- Motors reach target velocity within 0.5 seconds
- Velocity error < 50 ticks/sec at steady state
- Motor power ranges 0.6-0.9 (varies slightly)
- After shooting, velocity recovers within 0.5 seconds
- Power briefly spikes to 1.0 during recovery (OK!)
- Velocity stays constant during rapid fire

### Bad Signs ❌
- Motors run at 1.0 power constantly → F too low
- Velocity oscillates up/down → P or I too high
- Velocity never reaches target → F too low, or P too low
- Slow recovery after shooting → P too low, or I too low
- Velocity drops during rapid fire → I too low

---

## Troubleshooting

### "Motors always run at full power!"
- **Cause:** F gain is too low, P is saturating
- **Fix:**
  1. Increase F from 7.4 to 10.0
  2. Reduce P from 5.0 to 3.0
  3. Test and iterate

### "Velocity is unstable/oscillating!"
- **Cause:** P or I gain is too high
- **Fix:**
  1. Reduce P by 20%
  2. Reduce I to 0 temporarily
  3. Retune P, then add I back slowly

### "Shooter slows down when shooting rapidly!"
- **Cause:** Not enough I gain to fight load
- **Fix:**
  1. Increase I from 0.0 to 0.2
  2. Test rapid fire
  3. Increase to 0.5 if still slow

### "Velocity never reaches target!"
- **Cause:** Not enough total power (F + P × error)
- **Fix:**
  1. Increase F by 1.0
  2. Test again
  3. If still low, increase P

### "Recovery after shot is too slow!"
- **Cause:** P gain is too low
- **Fix:**
  1. Increase P by 1.0
  2. Watch for oscillations
  3. Add small I (0.1-0.2) to help

---

## Final Notes

1. **Both motors now use velocity control** - They will independently maintain target velocity
2. **Tune on the field** - Weight of balls, battery voltage, and wear affect performance
3. **Write down your values** - Update `RobotConstants.java` after tuning
4. **Test at all distances** - Make sure 2100, 2200, and 2600 velocities all work
5. **Battery voltage matters** - Retune if you notice performance changes with battery level

## Quick Reference Card

```
TUNING MODE CONTROLS:
Back Button    → Toggle tuning mode on/off
Y Button       → Switch between Spindexer/Shooter tuning
D-Pad L/R      → Select parameter (P/I/D/F)
D-Pad U/D      → Adjust selected parameter
B Button       → Change step size
X Button       → Test shot (trigger flipper)

TUNING ORDER:
1. Tune F first (with P=0)
2. Tune P second
3. Add I if needed
4. Test under load

TARGET VALUES:
Velocity Error: < 50 ticks/sec
Motor Power: 0.6-0.9 (not 1.0!)
Recovery Time: < 0.5 seconds
```

---

Good luck with tuning! Remember: **F is the most important gain for velocity control.**