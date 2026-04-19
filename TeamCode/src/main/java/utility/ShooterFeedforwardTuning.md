# Shooter Velocity Controller Tuning Guide

## Overview
The shooter uses a **dual-zone controller** for flywheel velocity:
- **RECOVERY mode**: Bang-bang (full power) — fastest possible spin-up after shots
- **MAINTAIN mode**: Feedforward + Proportional — accurate steady-state hold (±10-20 ticks)

All constants are in `ShooterConstants.java` and can be tuned live via Panels.

## How It Works

```
if (error > RECOVERY_THRESHOLD):
    RECOVERY: output = 1.0   (full power until near setpoint)
else:
    MAINTAIN: output = kS * sign(v) + kV * target + kP * error
```

- **kS**: Overcomes static friction (constant offset)
- **kV**: Maps target velocity to steady-state power (carries ~97% of the load)
- **kP**: Corrects residual error from kV miscalibration, voltage drift, temperature
- **RECOVERY_THRESHOLD**: Error boundary between full-power recovery and fine control
- **RECOVERY_VELOCITY_BOOST**: Overshoot buffer during recovery (high MOI absorbs it)

### Why Not PID?
After a shot, the velocity drops ~600 ticks. Any PID controller saturates at power = 1.0
during recovery (the old PIDFKSVA computed ~14.2 before clamping). PID gains are irrelevant
when the output is clamped. Bang-bang gives the same maximum power with zero tuning
complexity. The P-term only acts in MAINTAIN mode where errors are small and it provides
real value: steady-state accuracy across all voltages and velocities.

## Tuning Order

### Step 1: Calibrate kV (Most Important)
kV determines the steady-state power for any velocity. Get this right first.

1. Set `kS = 0`, `VELOCITY_kP = 0` (isolate kV)
2. Run the flywheel at power = 1.0 (use TUNING_MODE)
3. Measure the max velocity at full battery (~13V)
4. Calculate: `kV = 1.0 / maxVelocity`

**Example:** Max velocity = 2800 ticks/sec → `kV = 0.000357`

5. Set a target velocity (e.g., 2200 ticks/sec)
6. Verify steady-state holds within ~50 ticks (kV alone won't be perfect)
7. If consistently low, increase kV slightly. If high, decrease.

### Step 2: Set kS (Static Friction)
1. With kV set, try very low target velocities (~500 ticks/sec)
2. If the flywheel struggles to start, increase kS
3. Typical value: 0.03 - 0.08
4. kS should be just enough to overcome static friction

### Step 3: Add kP (Steady-State Accuracy)
kP corrects for kV imperfections across different velocities, temperatures, and voltages.

1. Start with `VELOCITY_kP = 0.005`
2. Set a target velocity and observe steady-state error on Panels
3. If error is > ±20 ticks, double kP
4. If you see oscillation (velocity wobbling around target), halve kP
5. Target: ±10-20 ticks at all LUT velocities (1380-2530 range)

**Math check:** With kP = 0.01, a 5% kV error at 2200 ticks/sec produces only
3.5 ticks of steady-state error. You likely don't need kP > 0.01.

### Step 4: Tune RECOVERY_THRESHOLD
This is the error boundary between RECOVERY (full power) and MAINTAIN (FF+P).

1. Default: `RECOVERY_THRESHOLD = 50.0` ticks/sec
2. Shoot and watch the `Flywheel Mode` field on Panels
3. If it toggles rapidly between RECOVERY/MAINTAIN near setpoint → increase threshold
4. If recovery is fast but you see a "bump" when transitioning to MAINTAIN → threshold is fine
5. Must be > `VELOCITY_TOLERANCE` (currently 10) to avoid chatter

### Step 5: Tune RECOVERY_VELOCITY_BOOST (Optional)
This keeps the motor at full power slightly past the real setpoint during recovery.
The flywheel's high MOI (from the 2x 0.5lb weights) absorbs the slight overshoot.

1. Start at `RECOVERY_VELOCITY_BOOST = 0` (no boost)
2. Measure recovery time after a shot
3. Increase by 25 ticks/sec increments
4. Watch for velocity ringing (overshooting then undershooting)
5. Stop at the value where recovery time plateaus without ringing
6. Typical range: 0-100 ticks/sec

## Telemetry Values to Monitor

```java
shooter.getCurrentVelocity()       // Actual flywheel velocity
shooter.getTargetVelocity()        // Desired velocity (from LUT)
shooter.getVelocityError()         // Target - Actual
shooter.isAtTargetVelocity()       // Within VELOCITY_TOLERANCE?
shooter.getFlywheelControlMode()   // RECOVERY or MAINTAIN
```

On Panels, watch:
- `Shooter Velocity (actual)` vs `(target)` — should track closely in MAINTAIN
- `Velocity Error` — should be < ±20 in steady-state
- `Flywheel Mode` — MAINTAIN in steady-state, RECOVERY after shots

## Constants Reference

```java
// Feedforward (steady-state)
kS = 0.03;                    // Static friction compensation
kV = 0.000305590469;          // Velocity gain (1 / max_velocity_at_nominal_voltage)

// Proportional (MAINTAIN mode only)
VELOCITY_kP = 0.01;           // Steady-state error correction

// Recovery tuning
RECOVERY_THRESHOLD = 50.0;    // Error threshold for RECOVERY mode (ticks/sec)
RECOVERY_VELOCITY_BOOST = 0.0; // Overshoot buffer during recovery (ticks/sec)

// Other
VELOCITY_TOLERANCE = 10.0;    // "At target" threshold for isAtTargetVelocity()
VOLTAGE_COMPENSATION_ENABLED = true;
NOMINAL_VOLTAGE = 13.0;       // Reference voltage for compensation
```

## Troubleshooting

| Problem | Likely Cause | Fix |
|---------|--------------|-----|
| Flywheel won't spin | kS too low | Increase kS |
| Steady-state velocity too low | kV too low | Recalibrate kV (Step 1) |
| Steady-state velocity too high | kV too high | Recalibrate kV (Step 1) |
| Steady-state error > ±20 ticks | kP too low or kV way off | Fix kV first, then increase kP |
| Oscillation at steady-state | kP too high | Decrease kP |
| Slow recovery after shot | Physics-limited (MOI + motor torque) | Reduce flywheel weights or increase RECOVERY_VELOCITY_BOOST |
| RECOVERY/MAINTAIN toggling | RECOVERY_THRESHOLD too low | Increase RECOVERY_THRESHOLD |
| Velocity ringing after recovery | RECOVERY_VELOCITY_BOOST too high | Decrease boost |
| Inconsistent across battery levels | Voltage compensation issue | Verify NOMINAL_VOLTAGE matches fresh battery |

## Physical Limits

Recovery speed is ultimately limited by:
- **Motor torque at operating speed** (~0.006 N·m available at 4700 RPM with 2 motors)
- **Flywheel moment of inertia** (increased ~3-4x by the 2x 0.5lb weights)
- **Available voltage headroom** (back-EMF consumes ~78% of voltage at operating speed)

No software controller can exceed maximum motor torque. Bang-bang guarantees you are
always at that maximum during recovery. If recovery is still too slow, the solution is
mechanical: reduce flywheel weight or move weights to a smaller radius (MOI ∝ r²).

## Quick Start Values

If starting from scratch:
```java
kS = 0.03
kV = 0.000357       // Recalibrate with Step 1!
VELOCITY_kP = 0.01
RECOVERY_THRESHOLD = 50.0
RECOVERY_VELOCITY_BOOST = 0.0
VELOCITY_TOLERANCE = 10.0
```
