package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.EnumConstants.FlickState;
import Constants.FieldMap;
import Constants.TurretConstants;
import utility.RobotHardware;
import Constants.ShooterConstants;
import Constants.ShootingSequenceConstants;

public class Shooter extends SubsystemBase {
    // Hardware reference
    RobotHardware robot;

    // Turret subsystem reference
    private Turret turret;

    // Flywheel velocity (ticks/sec) - updated each loop based on distance to goal
    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    private FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();



    // Custom feedforward + PID state
    private ElapsedTime loopTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastVelocityError = 0;
    private double lastVelocity = 0;

    // Default loop time when dt calculation fails (seconds)
    private static final double DEFAULT_LOOP_TIME = 0.02;

    // Cached velocity (updated once per periodic() call, reused by getters)
    private double cachedVelocity = 0;

    // Telemetry data for tuning
    private double lastFfOutput = 0;
    private double lastPidOutput = 0;
    private double lastTotalPower = 0;
    private double lastError = 0;
    private double lastAcceleration = 0;

    // InterpLUT instances for velocity, hood angle, and time-in-air lookup
    private InterpLUT velocityLUT;
    private InterpLUT hoodLUT;

    // Current required hood angle
    private double requiredHoodAngle = ShooterConstants.HOOD_DEFAULT_ANGLE;

    // Lead-compensated state (shared with Turret via getter)
    private double leadAdjustedDistance = ShooterConstants.DEFAULT_DISTANCE;
    private double currentTimeInAir = 0.0;
    private double[] futurePose = new double[3]; // {x, y, headingRad}

    // Hood servo dirty flag — only write when position changes
    private double lastHoodServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

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

        // Configure motors for EXTERNAL control (RUN_WITHOUT_ENCODER)
        // We still read the encoder for velocity feedback, but we control power directly
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);

        initializeLUTs();

        setHoodAngle(ShooterConstants.HOOD_DEFAULT_ANGLE);

        loopTimer.reset();
    }

    /**
     * Initialize InterpLUT instances from ShooterConstants data.
     */
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
        return velocityLUT.get(distance);
    }

    /**
     * Get required hood angle from distance using InterpLUT.
     * @param distance Distance to goal in inches
     * @return Hood angle in degrees (0=vertical, 90=horizontal)
     */
    private double getHoodAngleFromDistance(double distance) {
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

    /**
     * Set the hood to a specific angle, gated behind dirty flag.
     * @param angleDegrees Target angle (0=vertical, 90=horizontal)
     */
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

    /**
     * Get current target hood angle.
     */
    public double getTargetHoodAngle() {
        return requiredHoodAngle;
    }

    /**
     * Get time-in-air (constant for this robot).
     * @param distance Distance to goal in inches (unused, kept for API compatibility)
     * @return Flight time in seconds
     */
    public double getTimeInAirFromDistance(double distance) {
        return ShooterConstants.TIME_IN_AIR;
    }

    /**
     * Compute the predicted future robot pose based on current velocity and time-in-air.
     * Single-pass: getTimeInAirFromDistance() returns a constant, so refinement is unnecessary.
     */
    private void updateLeadCompensation() {
        double curX = robot.cachedPoseX;
        double curY = robot.cachedPoseY;
        double curH = robot.cachedHeading;
        double velX = robot.cachedVelX;
        double velY = robot.cachedVelY;
        double velH = robot.cachedHeadingVel;

        // Rotate Pinpoint robot-relative velocity into field-relative
        double cos = Math.cos(curH);
        double sin = Math.sin(curH);
        double fieldVelX = velX * cos - velY * sin;
        double fieldVelY = velX * sin + velY * cos;

        // Zero out noise-level velocities to prevent jitter when stationary
        double speed = Math.sqrt(fieldVelX * fieldVelX + fieldVelY * fieldVelY);
        if (speed < ShooterConstants.LEAD_VELOCITY_DEADBAND) {
            fieldVelX = 0.0;
            fieldVelY = 0.0;
        }
        if (Math.abs(velH) < ShooterConstants.LEAD_HEADING_VELOCITY_DEADBAND) {
            velH = 0.0;
        }

        // Single pass: timeInAir is constant, so refinement produces identical results
        double tof = ShooterConstants.TIME_IN_AIR;

        // Predict future pose
        futurePose[0] = curX + fieldVelX * tof;
        futurePose[1] = curY + fieldVelY * tof;
        futurePose[2] = curH + velH * tof;

        // Compute distance from future turret position using pre-computed polar offset
        double[] futureTurretPos = getTurretFieldPositionFrom(futurePose[0], futurePose[1], futurePose[2]);
        Pose goalPosition = FieldMap.getGoalPosition();
        double dx = goalPosition.getX() - futureTurretPos[0];
        double dy = goalPosition.getY() - futureTurretPos[1];
        leadAdjustedDistance = Math.sqrt(dx * dx + dy * dy);
        currentTimeInAir = tof;
    }

    /**
     * Compute turret field position from a hypothetical robot pose.
     * Uses pre-computed polar offset and returns pre-allocated array.
     */
    private double[] getTurretFieldPositionFrom(double robotX, double robotY, double robotHeadingRad) {
        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        turretFieldPosTemp[0] = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        turretFieldPosTemp[1] = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);
        return turretFieldPosTemp;
    }

    /**
     * Get the computed future pose for lead compensation.
     * @return {x, y, headingRad} of predicted robot position at ball arrival time
     */
    public double[] getFuturePose() {
        return futurePose;
    }

    private void updateVelocityFromDistance() {
        if (ShooterConstants.SHOOT_WHILE_MOVING_ENABLED) {
            updateLeadCompensation();
            requiredVelocity = getVelocityFromDistance(leadAdjustedDistance);
            requiredHoodAngle = getHoodAngleFromDistance(leadAdjustedDistance);
        } else {
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
    }

    public void retractFlipper() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
    }

    public FlickState getCurrentState() {
        return currentState;
    }


    /**
     * Clamp a value between min and max.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Check if flywheel is at target velocity (within tolerance).
     */
    public boolean isAtTargetVelocity() {
        return Math.abs(requiredVelocity - cachedVelocity) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Get current velocity error for telemetry.
     */
    public double getVelocityError() {
        return lastError;
    }

    /**
     * Get current velocity for telemetry.
     */
    public double getCurrentVelocity() {
        return cachedVelocity;
    }

    /**
     * Get target velocity for telemetry.
     */
    public double getTargetVelocity() {
        return requiredVelocity;
    }

    @Override
    public void periodic() {
        // Get loop time for derivative and integral calculations
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Prevent division by zero on first loop
        if (dt <= 0) dt = DEFAULT_LOOP_TIME;

        // Use manual tuning values when tuning mode is active, otherwise use LUT
        if (ShooterConstants.ShooterTuning.TUNING_MODE) {
            requiredVelocity = ShooterConstants.ShooterTuning.TUNING_VELOCITY;
            requiredHoodAngle = ShooterConstants.ShooterTuning.TUNING_HOOD_ANGLE;
        } else {
            updateVelocityFromDistance();
        }
        setHoodAngle(requiredHoodAngle);
        double targetVelocity = requiredVelocity;

        // Motor 1 encoder is dead — read motor 2 only until rebuild
        double currentVelocity = robot.shooterMotor2.getVelocity();
        cachedVelocity = currentVelocity;

        // Calculate velocity error
        double velocityError = targetVelocity - currentVelocity;
        lastError = velocityError;

        // Measure actual acceleration from velocity change
        double measuredAcceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;
        lastAcceleration = measuredAcceleration;

        // Calculate desired acceleration (aggressive: close the gap as fast as possible)
        // Clamp to physical limits
        double desiredAcceleration = velocityError / dt;
        desiredAcceleration = clamp(desiredAcceleration,
            -ShooterConstants.MAX_ACCELERATION,
            ShooterConstants.MAX_ACCELERATION);

        // ==================== FEEDFORWARD ====================
        // Full equation: kS * sign(v) + kV * targetVel + kA * acceleration
        double ffOutput = ShooterConstants.kS * Math.signum(targetVelocity)
                        + ShooterConstants.kV * targetVelocity
                        + ShooterConstants.kA * desiredAcceleration;
        lastFfOutput = ffOutput;

        // ==================== PID ====================
        // Proportional
        double pOutput = ShooterConstants.VELOCITY_kP * velocityError;

        // Integral with anti-windup
        integralSum += velocityError * dt;
        integralSum = clamp(integralSum, -ShooterConstants.INTEGRAL_MAX, ShooterConstants.INTEGRAL_MAX);

        // Zero-crossing reset prevents overshoot
        if (lastVelocityError != 0 && Math.signum(velocityError) != Math.signum(lastVelocityError)) {
            integralSum = 0;
        }
        double iOutput = ShooterConstants.VELOCITY_kI * integralSum;

        // Derivative (on measurement to avoid setpoint kick)
        double dOutput = ShooterConstants.VELOCITY_kD * -measuredAcceleration;

        lastVelocityError = velocityError;

        // Combine PID terms
        double pidOutput = pOutput + iOutput + dOutput;
        lastPidOutput = pidOutput;

        // ==================== TOTAL OUTPUT ====================
        double totalPower = ffOutput + pidOutput;
        totalPower = clamp(totalPower, 0, 1.0);  // Motor power range (flywheel only spins one direction)
        lastTotalPower = totalPower;

        // Apply power to both motors
        robot.shooterMotor1.setPower(totalPower);
        robot.shooterMotor2.setPower(totalPower);

        // Run flipper state machine
        flipperStateMachinePeriodic();
    }
}
