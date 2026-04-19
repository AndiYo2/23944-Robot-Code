package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants.FlickState;
import Constants.EnumConstants.FlywheelControlMode;
import Constants.FieldMap;
import Constants.TurretConstants;
import utility.RobotHardware;
import Constants.ShooterConstants;
import Constants.ShootingSequenceConstants;

public class Shooter extends SubsystemBase {
    RobotHardware robot;

    private Turret turret;

    /** Runtime velocity offset from gamepad Dpad. Stored here (not in @Configurable ShooterConstants) so Panels can't overwrite it. */
    public static double runtimeVelocityOffset = 0;

    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    private FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();



    private ElapsedTime loopTimer = new ElapsedTime();

    // Default loop time when dt calculation fails (seconds)
    private static final double DEFAULT_LOOP_TIME = 0.02;
    private double lastLoopDt = DEFAULT_LOOP_TIME;
    private double cachedVelocity = 0;
    private double lastFfOutput = 0;
    private double lastTotalPower = 0;
    private double lastError = 0;

    private FlywheelControlMode currentControlMode = FlywheelControlMode.MAINTAIN;

    private InterpLUT velocityLUT;
    private InterpLUT hoodLUT;

    private double requiredHoodAngle = ShooterConstants.HOOD_DEFAULT_ANGLE;

    private double currentTimeInAir = 0.0;

    // Shooting_While_Moving: position-derived velocity and acceleration tracking
    private double prevPoseX = 0.0;
    private double prevPoseY = 0.0;
    private double prevHeading = 0.0;
    private double prevFieldVelX = 0.0;
    private double prevFieldVelY = 0.0;
    private double prevHeadingVel = 0.0;
    private double filteredAccelX = 0.0;
    private double filteredAccelY = 0.0;
    private double filteredAngularAccel = 0.0;
    private boolean shootingWhileMovingInitialized = false;

    // Shooting_While_Moving output (shared with Turret via getters)
    private double shootingWhileMovingDistance = ShooterConstants.DEFAULT_DISTANCE;
    private double[] shootingWhileMovingFuturePose = new double[3]; // {x, y, headingRad}

    private double lastHoodServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

    // Cumulative hood compensation during shooting sequences
    private double shotHoodCompensation = 0.0;

    // Pre-allocated array for getTurretFieldPositionFrom() to avoid GC pressure
    private final double[] turretFieldPosTemp = new double[2];

    // Pre-computed turret offset in polar form (same as Turret class)
    private static final double TURRET_OFFSET_MAG = Math.sqrt(
            TurretConstants.TURRET_OFFSET_X * TurretConstants.TURRET_OFFSET_X +
            TurretConstants.TURRET_OFFSET_Y * TurretConstants.TURRET_OFFSET_Y);
    private static final double TURRET_OFFSET_ANGLE = Math.atan2(
            TurretConstants.TURRET_OFFSET_Y, TurretConstants.TURRET_OFFSET_X);


    public Shooter() {
        this.robot = RobotHardware.getInstance();

        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        initializeLUTs();

        loopTimer.reset();
    }

    public void initServoPositions() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
        setHoodAngle(ShooterConstants.HOOD_DEFAULT_ANGLE);
    }

    private void initializeLUTs() {
        velocityLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.VELOCITY_DATA) {
            velocityLUT.add(entry[0], entry[1]);
        }
        velocityLUT.createLUT();

        hoodLUT = new InterpLUT();
        for (double[] entry : ShooterConstants.HOOD_DATA) {
            hoodLUT.add(entry[0], entry[1]);
        }
        hoodLUT.createLUT();
    }

    public void setTurret(Turret turret) {
        this.turret = turret;
    }


    public double getDistanceToTarget() {
        if (turret == null) {
            return ShooterConstants.DEFAULT_DISTANCE;
        }
        double[] turretPos = turret.getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private double getVelocityFromDistance(double distance) {
        distance = Math.max(0, Math.min(ShooterConstants.LUT_MAX_DISTANCE, distance));
        return velocityLUT.get(distance) + ShooterConstants.VELOCITY_ADJUST_HARDCODED + runtimeVelocityOffset;
    }

    private double getHoodAngleFromDistance(double distance) {
        distance = Math.max(0, Math.min(ShooterConstants.LUT_MAX_DISTANCE, distance));
        double angle = hoodLUT.get(distance);

        return Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                       Math.min(ShooterConstants.HOOD_MAX_ANGLE, angle));
    }

    /**
     * Convert hood angle to servo position.
     * Servo 0.34 = 63 degrees (HOOD_MAX_ANGLE)
     * Servo 1.0 = 30 degrees (HOOD_MIN_ANGLE)
     *
     * @param hoodAngleDegrees Target hood angle in degrees
     * @return Servo position (0.34 to 1.0)
     */
    private double hoodAngleToServoPosition(double hoodAngleDegrees) {
        // Linear mapping: 30° → 1.0, 63° → 0.34
        double angleRange = ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE;
        double servoRange = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE;
        double normalizedAngle = (hoodAngleDegrees - ShooterConstants.HOOD_MIN_ANGLE) / angleRange;
        double position = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - (normalizedAngle * servoRange);

        position = Math.max(ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE,
                           Math.min(ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE, position));
        return position;
    }

    public void setHoodAngle(double angleDegrees) {
        angleDegrees = Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                               Math.min(ShooterConstants.HOOD_MAX_ANGLE, angleDegrees));

        double servoPosition = hoodAngleToServoPosition(angleDegrees);
        if (Math.abs(servoPosition - lastHoodServoPosition) > SERVO_EPSILON) {
            robot.shooterHood.setPosition(servoPosition);
            lastHoodServoPosition = servoPosition;
        }
        requiredHoodAngle = angleDegrees;
    }

    public double getTargetHoodAngle() {
        return requiredHoodAngle;
    }

    private double[] getTurretFieldPositionFrom(double robotX, double robotY, double robotHeadingRad) {
        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        turretFieldPosTemp[0] = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        turretFieldPosTemp[1] = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);
        return turretFieldPosTemp;
    }

    /**
     * Kinematic prediction with stop-clamp.
     * Computes pos + vel*t + 0.5*accel*t², but if deceleration would
     * reverse velocity direction before tof, clamps to the stopping point.
     */
    private double clampedPredict(double pos, double vel, double accel, double tof) {
        if (accel != 0.0 && vel != 0.0 && Math.signum(accel) != Math.signum(vel)) {
            double tStop = -vel / accel;
            if (tStop > 0 && tStop < tof) {
                return pos + vel * tStop + 0.5 * accel * tStop * tStop;
            }
        }
        return pos + vel * tof + 0.5 * accel * tof * tof;
    }

    /**
     * Shooting_While_Moving: Full kinematic shoot-while-moving compensation.
     *
     * Predicts future robot pose using: pos + vel*t + 0.5*accel*t²
     * Then computes turret angle, flywheel velocity, and hood angle
     * as if the robot were already at that predicted future position.
     *
     * Acceleration is derived from velocity differences between loops,
     * then low-pass filtered and deadbanded to reject sensor noise.
     */
    private void updateShootingWhileMovingCompensation() {
        double curX = robot.cachedPoseX;
        double curY = robot.cachedPoseY;
        double curH = robot.cachedHeading;

        double dt = lastLoopDt;

        // ---- INITIALIZATION ----
        if (!shootingWhileMovingInitialized) {
            prevPoseX = curX;
            prevPoseY = curY;
            prevHeading = curH;
            prevFieldVelX = 0.0;
            prevFieldVelY = 0.0;
            prevHeadingVel = 0.0;
            filteredAccelX = 0.0;
            filteredAccelY = 0.0;
            filteredAngularAccel = 0.0;
            shootingWhileMovingInitialized = true;

            shootingWhileMovingFuturePose[0] = curX;
            shootingWhileMovingFuturePose[1] = curY;
            shootingWhileMovingFuturePose[2] = curH;
            double[] futureTurretPos = getTurretFieldPositionFrom(curX, curY, curH);
            Pose goalPosition = FieldMap.getGoalPosition();
            double dx = goalPosition.getX() - futureTurretPos[0];
            double dy = goalPosition.getY() - futureTurretPos[1];
            shootingWhileMovingDistance = Math.sqrt(dx * dx + dy * dy);
            currentTimeInAir = ShooterConstants.TIME_IN_AIR;
            return;
        }

        // ---- VELOCITY FROM POSITION DELTAS ----
        // Positions are already field-relative, no rotation needed.
        double fieldVelX = (curX - prevPoseX) / dt;
        double fieldVelY = (curY - prevPoseY) / dt;
        double velH = (curH - prevHeading) / dt;

        // Update previous pose for next loop
        prevPoseX = curX;
        prevPoseY = curY;
        prevHeading = curH;

        // Save raw velocities BEFORE deadband for acceleration computation
        double rawFieldVelX = fieldVelX;
        double rawFieldVelY = fieldVelY;
        double rawVelH = velH;

        // Apply velocity deadbands — only affects the vel*t prediction term
        double speed = Math.sqrt(fieldVelX * fieldVelX + fieldVelY * fieldVelY);
        if (speed < ShooterConstants.LEAD_VELOCITY_DEADBAND) {
            fieldVelX = 0.0;
            fieldVelY = 0.0;
        }
        if (Math.abs(velH) < ShooterConstants.LEAD_HEADING_VELOCITY_DEADBAND) {
            velH = 0.0;
        }

        // ---- ACCELERATION COMPUTATION (uses raw velocities) ----
        {
            // --- COLLISION SPIKE REJECTION ---
            // If velocity changed more than physically possible in one loop,
            // it's a collision. Zero out acceleration but keep velocity intact.
            // The vel*t term still compensates for ball inheritance correctly.
            double dvx = rawFieldVelX - prevFieldVelX;
            double dvy = rawFieldVelY - prevFieldVelY;
            double dvMag = Math.sqrt(dvx * dvx + dvy * dvy);

            if (dvMag > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_VELOCITY_JUMP) {
                // Collision detected — don't trust acceleration this loop
                filteredAccelX = 0.0;
                filteredAccelY = 0.0;
                filteredAngularAccel = 0.0;
            } else {
                // Normal operation — compute and filter acceleration
                double rawAccelX = dvx / dt;
                double rawAccelY = dvy / dt;
                double rawAngularAccel = (rawVelH - prevHeadingVel) / dt;

                // Low-pass EMA filter
                double alpha = ShooterConstants.SHOOTING_WHILE_MOVING_ACCEL_FILTER_ALPHA;
                filteredAccelX = alpha * rawAccelX + (1.0 - alpha) * filteredAccelX;
                filteredAccelY = alpha * rawAccelY + (1.0 - alpha) * filteredAccelY;
                filteredAngularAccel = alpha * rawAngularAccel + (1.0 - alpha) * filteredAngularAccel;

                // Deadband: zero out negligible acceleration
                double accelMag = Math.sqrt(filteredAccelX * filteredAccelX + filteredAccelY * filteredAccelY);
                if (accelMag < ShooterConstants.SHOOTING_WHILE_MOVING_ACCEL_DEADBAND) {
                    filteredAccelX = 0.0;
                    filteredAccelY = 0.0;
                }
                if (Math.abs(filteredAngularAccel) < ShooterConstants.SHOOTING_WHILE_MOVING_ANGULAR_ACCEL_DEADBAND) {
                    filteredAngularAccel = 0.0;
                }
            }

            // --- HARD ACCELERATION CAP ---
            // Even after filtering, clamp to physical robot limits.
            double accelMag = Math.sqrt(filteredAccelX * filteredAccelX + filteredAccelY * filteredAccelY);
            if (accelMag > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ACCEL) {
                double scale = ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ACCEL / accelMag;
                filteredAccelX *= scale;
                filteredAccelY *= scale;
            }
            if (Math.abs(filteredAngularAccel) > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL) {
                filteredAngularAccel = Math.signum(filteredAngularAccel) * ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL;
            }

            // Store raw velocity for next loop's diff (always update, even during collision)
            prevFieldVelX = rawFieldVelX;
            prevFieldVelY = rawFieldVelY;
            prevHeadingVel = rawVelH;
        }

        // ---- FULL KINEMATIC PREDICTION (clamped to stop point) ----
        double tof = ShooterConstants.TIME_IN_AIR;

        shootingWhileMovingFuturePose[0] = clampedPredict(curX, fieldVelX, filteredAccelX, tof);
        shootingWhileMovingFuturePose[1] = clampedPredict(curY, fieldVelY, filteredAccelY, tof);
        shootingWhileMovingFuturePose[2] = clampedPredict(curH, velH, filteredAngularAccel, tof);

        // ---- DISTANCE FROM FUTURE TURRET POSITION TO GOAL ----
        double[] futureTurretPos = getTurretFieldPositionFrom(
                shootingWhileMovingFuturePose[0], shootingWhileMovingFuturePose[1], shootingWhileMovingFuturePose[2]);
        Pose goalPosition = FieldMap.getGoalPosition();
        double dx = goalPosition.getX() - futureTurretPos[0];
        double dy = goalPosition.getY() - futureTurretPos[1];
        shootingWhileMovingDistance = Math.sqrt(dx * dx + dy * dy);
        currentTimeInAir = tof;
    }

    /**
     * Returns the Shooting_While_Moving predicted future pose {x, y, headingRad}.
     * Used by Turret to aim from the predicted position.
     */
    public double[] getShootingWhileMovingFuturePose() {
        return shootingWhileMovingFuturePose;
    }

    /**
     * Returns the Shooting_While_Moving distance from predicted future turret position to goal.
     */
    public double getShootingWhileMovingDistance() {
        return shootingWhileMovingDistance;
    }

    private void updateVelocityFromDistance() {
        if (ShooterConstants.SHOOTING_WHILE_MOVING_ENABLED) {
            updateShootingWhileMovingCompensation();
            requiredVelocity = getVelocityFromDistance(shootingWhileMovingDistance);
            requiredHoodAngle = getHoodAngleFromDistance(shootingWhileMovingDistance);
        } else {
            shootingWhileMovingInitialized = false;
            double distance = getDistanceToTarget();
            requiredVelocity = getVelocityFromDistance(distance);
            requiredHoodAngle = getHoodAngleFromDistance(distance);
        }
    }

    private void flipperStateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < ShootingSequenceConstants.SHOOTER_FLICK_TIME) break;
                robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                if (flickerTimer.seconds() < ShootingSequenceConstants.SHOOTER_RETRACT_DELAY) break;
                currentState = FlickState.Idle;
                break;
        }
    }

    public void triggerShot() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }

    public void extendFlipper() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_EXTENDED);
        shotHoodCompensation += ShooterConstants.ShooterTuning.SHOT_HOOD_COMPENSATION_STEP;
    }

    public void resetHoodCompensation() {
        shotHoodCompensation = 0.0;
    }

    public void retractFlipper() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
    }

    public boolean isFlipperAtPosition(double targetPosition) {
        double actual = robot.shooterFlipperEncoder.getVoltage() / 3.3;
        return Math.abs(actual - targetPosition) < ShootingSequenceConstants.SHOOTER_FLIPPER_POSITION_TOLERANCE;
    }

    public FlickState getCurrentState() {
        return currentState;
    }


    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public boolean isAtTargetVelocity() {
        return Math.abs(requiredVelocity - cachedVelocity) < ShooterConstants.ShooterTuning.VELOCITY_TOLERANCE;
    }

    public double getVelocityError() {
        return lastError;
    }

    public double getShotHoodCompensation() {
        return shotHoodCompensation;
    }

    public FlywheelControlMode getFlywheelControlMode() {
        return currentControlMode;
    }


    public double getCurrentVelocity() {
        return cachedVelocity;
    }

    public double getTargetVelocity() {
        return requiredVelocity;
    }

    @Override
    public void periodic() {
        // Get loop time for derivative and integral calculations
        double dt = loopTimer.seconds();
        loopTimer.reset();

        if (dt <= 0) dt = DEFAULT_LOOP_TIME;
        lastLoopDt = dt;

        if (ShooterConstants.MANUAL_OVERRIDE) {
            requiredVelocity = ShooterConstants.MANUAL_OVERRIDE_VELOCITY + ShooterConstants.VELOCITY_ADJUST_HARDCODED + runtimeVelocityOffset;
            requiredHoodAngle = ShooterConstants.MANUAL_OVERRIDE_HOOD;
            shootingWhileMovingFuturePose[0] = robot.cachedPoseX;
            shootingWhileMovingFuturePose[1] = robot.cachedPoseY;
            shootingWhileMovingFuturePose[2] = robot.cachedHeading;
        } else if (ShooterConstants.ShooterTuning.TUNING_MODE) {
            requiredVelocity = ShooterConstants.ShooterTuning.TUNING_VELOCITY;
            requiredHoodAngle = ShooterConstants.ShooterTuning.TUNING_HOOD_ANGLE;
            shootingWhileMovingFuturePose[0] = robot.cachedPoseX;
            shootingWhileMovingFuturePose[1] = robot.cachedPoseY;
            shootingWhileMovingFuturePose[2] = robot.cachedHeading;
        } else {
            updateVelocityFromDistance();
        }
        setHoodAngle(requiredHoodAngle + shotHoodCompensation);
        double targetVelocity = requiredVelocity;

        //  read motor 2 only until rebuild
        double currentVelocity = robot.shooterMotor2.getVelocity();
        cachedVelocity = currentVelocity;

        double velocityError = targetVelocity - currentVelocity;
        lastError = velocityError;

        // ==================== BANG-BANG RECOVERY + FF+P MAINTAIN ====================
        double totalPower;

        if (velocityError > ShooterConstants.ShooterTuning.RECOVERY_THRESHOLD) {
            // --- RECOVERY MODE: full power for fastest possible spin-up ---
            currentControlMode = FlywheelControlMode.RECOVERY;
            double effectiveTarget = targetVelocity + ShooterConstants.ShooterTuning.RECOVERY_VELOCITY_BOOST;

            if (currentVelocity < effectiveTarget) {
                totalPower = 1.0;
            } else {
                // Boost pushed us past effective target — use feedforward
                double ff = ShooterConstants.ShooterTuning.kS * Math.signum(targetVelocity)
                          + ShooterConstants.ShooterTuning.kV * targetVelocity;
                totalPower = ff;
            }
            lastFfOutput = totalPower;
        } else {
            // --- MAINTAIN MODE: feedforward + proportional for steady-state accuracy ---
            currentControlMode = FlywheelControlMode.MAINTAIN;
            double ff = ShooterConstants.ShooterTuning.kS * Math.signum(targetVelocity)
                      + ShooterConstants.ShooterTuning.kV * targetVelocity;
            double pCorrection = ShooterConstants.ShooterTuning.VELOCITY_kP * velocityError;
            totalPower = ff + pCorrection;
            lastFfOutput = ff;
        }

        totalPower = clamp(totalPower, 0, 1.0);

        // Voltage compensation: scale power up as battery sags below nominal
        if (ShooterConstants.ShooterTuning.VOLTAGE_COMPENSATION_ENABLED && robot.voltageSensor != null) {
            double batteryVoltage = robot.voltageSensor.getVoltage();
            if (batteryVoltage > 0) {
                totalPower *= ShooterConstants.ShooterTuning.NOMINAL_VOLTAGE / batteryVoltage;
                totalPower = clamp(totalPower, 0, 1.0);
            }
        }

        lastTotalPower = totalPower;

        robot.shooterMotor1.setPower(totalPower);
        robot.shooterMotor2.setPower(totalPower);

        flipperStateMachinePeriodic();
    }
}
