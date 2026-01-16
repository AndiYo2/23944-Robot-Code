# Shoot-While-Moving System Design

## Requirements
- Adjustable hood (30-63°) AND adjustable velocity
- Shoot while robot is moving at any speed
- Compensate for robot velocity in aiming
- Zero tolerance for misses

---

## Physics: Shooting While Moving

When the robot moves, the ball inherits that velocity. You must compensate.

### Vector Components

Robot velocity relative to target breaks into:
1. **Radial (toward/away from target)**: Affects required exit velocity
2. **Tangential (perpendicular to target)**: Affects turret aim angle

```
Target
   X
   |\
   | \  tangential component (vT)
   |  \
   |   \  robot velocity vector
   |    \
   |_____\
   radial component (vR)

   Robot
```

### Compensation Formulas

**1. Velocity Compensation (radial component):**
```
vR = robotVelocity * cos(angleToTarget - robotHeading)

adjustedExitVelocity = baseExitVelocity - vR
```
- Robot moving toward target → reduce exit velocity
- Robot moving away from target → increase exit velocity

**2. Turret Angle Compensation (tangential component):**
```
vT = robotVelocity * sin(angleToTarget - robotHeading)

leadAngle = atan(vT / baseExitVelocity)

adjustedTurretAngle = baseAngle + leadAngle
```
- Robot moving left relative to target → aim right (lead the shot)
- Robot moving right relative to target → aim left

**3. Hood Angle:**
```
Since total ball velocity changes, recalculate hood angle for new effective velocity
```

---

## Implementation

### Inputs Needed Each Cycle
- `robotX, robotY` - from odometry
- `robotVx, robotVy` - robot velocity (from localizer)
- `robotHeading` - current heading
- `targetX, targetY` - goal position

### Calculation Steps

```java
// 1. Calculate distance and angle to target
double dx = targetX - robotX;
double dy = targetY - robotY;
double distance = Math.sqrt(dx*dx + dy*dy);
double angleToTarget = Math.atan2(dy, dx);

// 2. Get robot velocity magnitude and direction
double robotSpeed = Math.sqrt(robotVx*robotVx + robotVy*robotVy);
double robotVelocityAngle = Math.atan2(robotVy, robotVx);

// 3. Decompose robot velocity relative to target
double relativeAngle = angleToTarget - robotVelocityAngle;
double vRadial = robotSpeed * Math.cos(relativeAngle);    // toward/away
double vTangential = robotSpeed * Math.sin(relativeAngle); // perpendicular

// 4. Get base parameters from lookup tables
double baseVelocity = lookupVelocity(distance);
double baseHoodAngle = lookupHoodAngle(distance);

// 5. Adjust for robot motion
double adjustedVelocity = baseVelocity - vRadial;
double leadAngle = Math.atan2(vTangential, baseVelocity);
double adjustedTurretAngle = angleToTarget + leadAngle - robotHeading;

// 6. Recalculate hood angle for adjusted velocity
double adjustedHoodAngle = lookupHoodAngle(distance, adjustedVelocity);
```

---

## Lookup Table Structure

You need a 2D lookup or interpolation:

**Option A: 2D Table (distance × velocity)**
```
Distance | Velocity | Hood Angle
---------|----------|----------
48"      | 1900     | 58°
48"      | 2100     | 52°
72"      | 2000     | 50°
72"      | 2200     | 45°
...
```

**Option B: Base table + velocity adjustment curve**
```
baseHoodAngle = lookupByDistance(distance)
velocityDelta = adjustedVelocity - baseVelocity
hoodAdjustment = velocityDelta * HOOD_VELOCITY_COEFFICIENT
finalHoodAngle = baseHoodAngle + hoodAdjustment
```

---

## Calibration Process

1. **Static calibration first**: Robot stationary, build distance → (velocity, hoodAngle) table
2. **Velocity compensation tuning**: Drive toward/away from target at known speeds, adjust vRadial coefficient
3. **Lead angle tuning**: Drive perpendicular to target, adjust leadAngle coefficient
4. **Combined testing**: Drive in circles/arcs while shooting, verify accuracy

---

## Critical Implementation Details

### Velocity Measurement
Your Pedro Pathing localizer provides velocity directly:
```java
Pose2D velocity = follower.poseTracker.getLocalizer().getVelocity();
double robotVx = velocity.getX();  // inches/sec
double robotVy = velocity.getY();  // inches/sec
```
Apply exponential moving average filter to smooth noise.

### Latency Compensation (Critical for accuracy)
Ball exit is not instant. You must predict where robot will be when ball leaves:

```java
// Measure these values for your robot
static final double BALL_EXIT_DELAY = 0.080; // seconds from trigger to ball leaving shooter

// In calculation:
double predictedX = robotX + robotVx * BALL_EXIT_DELAY;
double predictedY = robotY + robotVy * BALL_EXIT_DELAY;
// Use predicted position for all distance/angle calculations
```

Also account for hood servo movement time if significant.

### Safety Bounds
- Clamp all outputs to safe ranges
- If calculated velocity exceeds safe limits → don't shoot
- If lead angle exceeds turret range → don't shoot

---

## Files to Modify

1. `Shooter.java` - Add velocity adjustment logic
2. `ShooterConstants.java` - Add hood angle lookup tables
3. `Turret.java` - Add lead angle compensation
4. Create new `ShootingCalculator.java` - Centralize all ballistic math
5. `TeleOp.java` - Integrate new shooting system

---

## Testing Verification

1. Static shots at 2ft increments (0-12ft) - all must hit
2. Driving toward target at 1, 2, 3 ft/s - all must hit
3. Driving away from target at 1, 2, 3 ft/s - all must hit
4. Driving perpendicular at 1, 2, 3 ft/s - all must hit
5. Driving in arc patterns - all must hit
