# Shooting On The Move (SOTM) Implementation Plan

## Analysis of Decode 2026's Approach

After analyzing FTC-23511's `sotm` branch, here's how their system works:

### Key Components

1. **ShootingMath.predict()** - The core algorithm that:
   - Takes current robot pose and velocity (ChassisSpeeds)
   - Calculates ball flight time based on distance and gravity
   - Compensates turret angle for robot translational velocity
   - Adjusts flywheel speed to account for relative motion
   - Returns: turret angle, hood angle, flywheel velocity

2. **Turret Subsystem** with velocity feedforward:
   - Uses `kVelocityFeedforward * driveRotationalVelocity` to compensate for robot rotation
   - Distributes rotation between drivetrain and turret for optimal tracking

3. **MovingAim Command**:
   - Continuously calls `predictSet()` in execute() loop
   - Updates turret, hood, and flywheel in real-time
   - Monitors flywheel stability before allowing launch

---

## Implementation Plan for Your Codebase

### Phase 1: Add Velocity Tracking to Odometry

**File: `subsystems/Odometry.java`**

Add methods to track robot velocity using Pinpoint:
```java
// Add these fields
private double lastX, lastY, lastHeading;
private double velocityX, velocityY, angularVelocity; // inches/sec, rad/sec
private ElapsedTime velocityTimer;

// Add velocity calculation in periodic()
public void updateVelocity() {
    Pose2D currentPose = robot.pinpoint.getPosition();
    double dt = velocityTimer.seconds();
    velocityTimer.reset();

    if (dt > 0 && dt < 0.5) { // Sanity check
        velocityX = (currentPose.getX(DistanceUnit.INCH) - lastX) / dt;
        velocityY = (currentPose.getY(DistanceUnit.INCH) - lastY) / dt;
        angularVelocity = (currentPose.getHeading(AngleUnit.RADIANS) - lastHeading) / dt;
    }

    lastX = currentPose.getX(DistanceUnit.INCH);
    lastY = currentPose.getY(DistanceUnit.INCH);
    lastHeading = currentPose.getHeading(AngleUnit.RADIANS);
}

public double[] getVelocity() {
    return new double[]{velocityX, velocityY, angularVelocity};
}
```

---

### Phase 2: Create ShootingMath Utility Class

**New File: `utility/ShootingMath.java`**

Core ballistics calculations accounting for robot motion:

```java
public class ShootingMath {
    // Physics constants
    private static final double GRAVITY = 386.1; // in/s^2
    private static final double LAUNCHER_HEIGHT = 13.0; // inches
    private static final double TARGET_HEIGHT = 39.4; // inches (goal rim)

    /**
     * Calculates shooting parameters while robot is moving.
     *
     * @param turretPos field position of turret [x, y] in inches
     * @param robotHeading robot heading in radians
     * @param velocityX robot X velocity in inches/sec (field frame)
     * @param velocityY robot Y velocity in inches/sec (field frame)
     * @param angularVel robot angular velocity in rad/sec
     * @param goalPos goal position [x, y] in inches
     * @return [turretAngleDeg, hoodAngleDeg, flywheelVelocity, isValid]
     */
    public static double[] predictShot(
            double[] turretPos, double robotHeading,
            double velocityX, double velocityY, double angularVel,
            double[] goalPos) {

        // Vector from turret to goal
        double dx = goalPos[0] - turretPos[0];
        double dy = goalPos[1] - turretPos[1];
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Calculate flight time estimate (iterative refinement)
        double flightTime = estimateFlightTime(distance);

        // Predicted goal position relative to where turret will be
        // (accounting for robot movement during ball flight)
        double predictedDx = dx - velocityX * flightTime;
        double predictedDy = dy - velocityY * flightTime;
        double predictedDistance = Math.sqrt(predictedDx * predictedDx + predictedDy * predictedDy);

        // Field angle to predicted position
        double fieldAngle = Math.atan2(predictedDy, predictedDx);

        // Account for robot rotation during flight time
        double predictedHeading = robotHeading + angularVel * flightTime;

        // Convert to robot-relative turret angle
        double turretAngleRad = fieldAngle - predictedHeading;
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // Get hood angle and velocity from distance (use your existing LUTs)
        // Adjust velocity slightly based on robot speed towards/away from goal
        double radialVelocity = (dx * velocityX + dy * velocityY) / distance;

        // Velocity adjustment: if moving toward goal, reduce flywheel speed slightly
        double velocityAdjustment = -radialVelocity * 0.5; // tune this factor

        boolean isValid = Math.abs(turretAngleDeg) <= 60; // within turret limits

        return new double[]{turretAngleDeg, predictedDistance, velocityAdjustment, isValid ? 1.0 : 0.0};
    }

    private static double estimateFlightTime(double distance) {
        // Simple physics estimate: t = d / v_horizontal
        // Assuming ~400 in/sec average ball velocity
        return distance / 400.0;
    }

    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        else if (degrees < -180) degrees += 360;
        return degrees;
    }
}
```

---

### Phase 3: Add Velocity Feedforward to Turret

**File: `subsystems/Turret.java`**

Add angular velocity compensation:

```java
// New constant in TurretConstants.java
public static double ANGULAR_VELOCITY_FEEDFORWARD = 0.5; // tune this

// Modified getDegreesToGoal() with velocity compensation
public double getDegreesToGoalWithVelocity(double angularVelocity, double lookahead) {
    // Get static angle to goal
    double staticAngle = getDegreesToGoal();

    // Compensate for robot rotation during servo response time
    // If robot is rotating CW (positive), turret needs to rotate CCW (negative) to compensate
    double velocityCompensation = -angularVelocity * lookahead *
                                  TurretConstants.ANGULAR_VELOCITY_FEEDFORWARD;

    return staticAngle + Math.toDegrees(velocityCompensation);
}
```

---

### Phase 4: Create MovingAimCommand

**New File: `commands/MovingAimCommand.java`**

```java
public class MovingAimCommand extends CommandBase {
    private final Turret turret;
    private final Shooter shooter;
    private final Odometry odometry;

    private static final double LOOKAHEAD_TIME = 0.1; // seconds

    public MovingAimCommand(Turret turret, Shooter shooter, Odometry odometry) {
        this.turret = turret;
        this.shooter = shooter;
        this.odometry = odometry;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // Get robot velocity
        double[] velocity = odometry.getVelocity();
        double angularVel = velocity[2];

        // Calculate turret angle with velocity compensation
        double compensatedAngle = turret.getDegreesToGoalWithVelocity(angularVel, LOOKAHEAD_TIME);
        turret.setTurretAngle(compensatedAngle);

        // Optionally adjust shooter parameters based on movement
        // (for now, let existing distance-based LUT handle this)
    }

    @Override
    public boolean isFinished() {
        return false; // Runs until interrupted
    }
}
```

---

### Phase 5: Create MovingShootCommand Sequence

**New File: `commands/MovingShootCommand.java`**

```java
public class MovingShootCommand extends SequentialCommandGroup {
    public MovingShootCommand(Turret turret, Shooter shooter,
                              Odometry odometry, Spindexer spindexer) {
        addCommands(
            // Start moving aim tracking
            new MovingAimCommand(turret, shooter, odometry)
                .raceWith(
                    new SequentialCommandGroup(
                        // Wait for flywheel and turret to be ready
                        new WaitForShooterReadyCommand(shooter, 2.0),
                        new WaitForTurretAlignedCommand(turret, 1.0),
                        // Fire all balls
                        ShootingCommands.shootThreeBalls(shooter, spindexer)
                    )
                )
        );
    }
}
```

---

### Phase 6: Integrate into CommandSequenceBuilder

**File: `commands/CommandSequenceBuilder.java`**

Add new methods:

```java
/**
 * Shoots while the robot is moving, compensating for velocity.
 * Uses MovingAim to continuously track the goal during robot motion.
 */
public CommandSequenceBuilder shootWhileMoving() {
    commands.add(new MovingShootCommand(turret, shooter, odometry, spindexer));
    return this;
}

/**
 * Enables moving aim mode (turret continuously tracks with velocity compensation).
 * Combine with path following in a parallel block.
 */
public CommandSequenceBuilder enableMovingAim() {
    commands.add(new MovingAimCommand(turret, shooter, odometry));
    return this;
}
```

And in ParallelBuilder:
```java
public ParallelBuilder shootWhileMoving() {
    parallelCommands.add(new MovingShootCommand(turret, shooter, odometry, spindexer));
    return this;
}
```

---

## Usage Example

```java
// Shoot while driving to next position
autonomousCommand = new CommandSequenceBuilder(...)
    .preload()
    .parallel(p -> p
        .moveTo(pathToSecondPosition)
        .shootWhileMoving()  // NEW: shoots while moving
    )
    .intakeStart()
    .moveTo(intakePath)
    .build();
```

---

## Tuning Steps

1. **Test velocity tracking**: Verify `odometry.getVelocity()` returns accurate values
2. **Tune ANGULAR_VELOCITY_FEEDFORWARD**: Start at 0.3, increase if turret lags behind rotation
3. **Tune LOOKAHEAD_TIME**: Higher values = more aggressive prediction, but more overshoot
4. **Test at various speeds**: Walk before you run - test at 30%, 50%, 80% path speed
5. **Refine ShootingMath**: Adjust flight time estimation based on actual ball speed data

---

## Files to Create/Modify

| File | Action |
|------|--------|
| `utility/ShootingMath.java` | CREATE |
| `commands/MovingAimCommand.java` | CREATE |
| `commands/MovingShootCommand.java` | CREATE |
| `subsystems/Odometry.java` | MODIFY - add velocity tracking |
| `subsystems/Turret.java` | MODIFY - add velocity feedforward |
| `Constants/TurretConstants.java` | MODIFY - add feedforward constant |
| `commands/CommandSequenceBuilder.java` | MODIFY - add new builder methods |

---

## Key Differences from Decode 2026

1. **Simplified approach**: They use continuous rotation motor with PID; you use position servo
2. **No full ballistics**: Their ShootingMath does full projectile physics; this plan starts simpler
3. **Incremental**: Start with angular velocity compensation only, add translational later

---

## Recommended Implementation Order

1. Add velocity tracking to Odometry (required foundation)
2. Test velocity accuracy with telemetry
3. Add basic angular velocity feedforward to Turret
4. Create MovingAimCommand and test in TeleOp
5. Create MovingShootCommand for autonomous
6. Add full ShootingMath with translational compensation (advanced)
7. Integrate into CommandSequenceBuilder
8. Tune, tune, tune!