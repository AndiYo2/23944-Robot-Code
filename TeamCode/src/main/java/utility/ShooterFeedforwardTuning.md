# Shooter Feedforward Tuning Guide

## Overview
The shooter uses a custom feedforward + PID controller for velocity control.
All constants are in `ShooterConstants.java` and can be tuned live via panels.

**Goal:** Fastest possible velocity recovery after shooting (target: <200ms, achievable: 50-150ms)

## The Control Equation
```
output = kS * sign(v) + kV * targetVelocity + kA * acceleration + PID_correction
```

- **kS**: Overcomes static friction
- **kV**: Steady-state velocity mapping
- **kA**: Acceleration compensation (the key to fast recovery)
- **PID**: Corrects remaining error

## Tuning Order (IMPORTANT)

### Step 1: Find kV (Most Important)
kV determines the steady-state power for any velocity.

1. Set all gains to 0: `kS=0, kV=0, kA=0, VELOCITY_kP=0, VELOCITY_kI=0, VELOCITY_kD=0`
2. Set `kS=0.05` (rough static friction estimate)
3. Run the flywheel at power = 1.0
4. Measure the max velocity (should be ~2800 ticks/sec)
5. Calculate: `kV = 1.0 / maxVelocity`

**Example:** If max velocity = 2800 ticks/sec, then `kV = 0.000357`

### Step 2: Verify Steady-State
1. Set a target velocity (e.g., 2200 ticks/sec)
2. The flywheel should spin up and hold near target
3. If velocity is low, slightly increase kV
4. If velocity is high, slightly decrease kV

### Step 3: Find kS (Static Friction)
kS is the minimum power to overcome friction.

1. With kV set, slowly decrease target velocity toward 0
2. Watch when the flywheel stops responding
3. Adjust kS so very low velocities still work
4. Typical value: 0.03 - 0.08

### Step 4: Tune kA (Acceleration - Key for Recovery)
kA enables fast recovery after shooting.

1. Run flywheel at target velocity (2200 ticks/sec)
2. Trigger a shot and watch recovery time
3. If recovery is slow (>200ms), increase kA
4. If you see overshoot, decrease kA
5. Start at `kA = 0.00001`, increase by 2x until fast

**Target:** Recovery in 50-150ms without overshoot

### Step 5: Add VELOCITY_kP (Fine Correction)
Only if steady-state error persists after feedforward tuning.

1. Start with `VELOCITY_kP = 0.0001`
2. Small increments (0.00005) if needed
3. Too high = oscillation

### Step 6: Add VELOCITY_kI (Steady-State Error)
Only if there's consistent steady-state error that kP can't fix.

1. Start with `VELOCITY_kI = 0.0001`
2. Very small increments
3. Too high = overshoot and slow oscillation

### Step 7: VELOCITY_kD (Usually Not Needed)
Derivative dampens oscillation but is rarely needed for velocity control.

1. Usually leave at 0
2. Only add if you see oscillation that kP reduction doesn't fix

## Telemetry Values to Monitor

Use these methods in your TeleOp for tuning:
```java
shooter.getCurrentVelocity()    // Actual flywheel velocity
shooter.getTargetVelocity()     // Desired velocity
shooter.getVelocityError()      // Target - Actual
shooter.isAtTargetVelocity()    // Within tolerance?

// Detailed breakdown:
double[] outputs = shooter.getControllerOutputs();
// outputs[0] = ffOutput (feedforward contribution)
// outputs[1] = pidOutput (PID contribution)
// outputs[2] = totalPower (final motor power)
// outputs[3] = acceleration (measured acceleration)
```

## Constants Reference

```java
// Feedforward
kS = 0.05;          // Static friction
kV = 0.00035;       // Velocity gain
kA = 0.00001;       // Acceleration gain (CRITICAL FOR FAST RECOVERY)

// PID
VELOCITY_kP = 0.0001;
VELOCITY_kI = 0.0002;
VELOCITY_kD = 0.0;

// Limits
INTEGRAL_MAX = 0.3;           // Anti-windup
MAX_ACCELERATION = 15000.0;   // Physical limit (ticks/sec^2)
VELOCITY_TOLERANCE = 50.0;    // "At target" threshold
```

## Troubleshooting

| Problem | Likely Cause | Fix |
|---------|--------------|-----|
| Flywheel won't spin | kS too low | Increase kS |
| Velocity too low | kV too low | Increase kV |
| Velocity too high | kV too high | Decrease kV |
| Slow recovery | kA too low | Increase kA |
| Overshoot after shot | kA too high or kI too high | Decrease kA, reset integral |
| Oscillation | kP or kI too high | Decrease gains |
| Steady-state error | kV wrong or needs kI | Tune kV first, then add small kI |

## Physical Limits

Your recovery speed is ultimately limited by:
- Motor torque
- Flywheel moment of inertia
- Available voltage headroom (battery state)

With proper feedforward, you should be limited by physics, not software.
Typical well-tuned flywheel: **50-150ms recovery** for a ~300 ticks/sec drop.

## Quick Start Values

If starting from scratch:
```java
kS = 0.05
kV = 0.000357  // Assuming 2800 max velocity
kA = 0.00002
VELOCITY_kP = 0.0001
VELOCITY_kI = 0.0001
VELOCITY_kD = 0
```

Then tune kA aggressively for fastest recovery without overshoot.
