package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import utility.FieldMap;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FieldState;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;

import static utility.RobotConstants.Shooter.CENTER;
import static utility.RobotConstants.Shooter.FLICK_TIME;

public class Shooter implements Subsystem {
    // Hardware reference
    RobotHardware robot;

    // Flywheel velocity (ticks/sec) - updated each loop based on distance to goal
    private double requiredVelocity = 2200;

    // Ball flipper state machine
    FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();

    // Field zone state - determines whether turret tracks goal or stays centered
    public FieldState fieldState = FieldState.IdleZone;

    // Manual velocity override (for tuning/testing)
    private boolean manualVelocityMode = false;
    private double manualVelocity = 2200;

    // Turret PID state variables (all angles in SERVO degrees, not turret degrees)
    private double targetTurretAngle = 0.0;
    private double lastTargetTurretAngle = 0.0;
    private double lastTurretError = 0;
    private double turretIntegral = 0;
    private long lastTurretTime = 0;

    // Turret PID gains (can be updated via setTurretPID for tuning)
    private double turretKP = RobotConstants.Shooter.TURRET_PID.p;
    private double turretKI = RobotConstants.Shooter.TURRET_PID.i;
    private double turretKD = RobotConstants.Shooter.TURRET_PID.d;

    // Cumulative position tracking for turret encoder
    // Needed because turret can rotate beyond 360° (servo range: -360° to +270°)
    private double lastRawTurretPosition = 0;
    private double cumulativeTurretPosition = 0;
    private int turretRotationCount = 0;

    // Limelight reference for switching between goal tracking and tag scanning modes
    private subsystems.Limelight limelightSubsystem;

    // ==================== DIAGNOSTIC DATA ====================
    // Stored values from last angle calculation (for debugging turret tracking)
    private double diagRobotX = 0;
    private double diagRobotY = 0;
    private double diagRobotHeading = 0;
    private double diagTurretX = 0;
    private double diagTurretY = 0;
    private double diagGoalX = 0;
    private double diagGoalY = 0;
    private double diagDeltaX = 0;
    private double diagDeltaY = 0;
    private double diagAbsoluteAngle = 0;
    private double diagRelativeAngle = 0;
    private double diagTurretError = 0;

    // Out-of-range tracking - set when target angle exceeds hardware limits
    private boolean targetOutOfRange = false;
    private double degreesOutOfRange = 0;  // How far beyond limit (positive value)

    // Drift detection - tracks if cumulative position seems to have drifted
    private double lastRawForDrift = 0;       // Last raw reading for drift check
    private double expectedCumulative = 0;    // What cumulative should be based on deltas
    private boolean driftDetected = false;
    private double driftAmount = 0;           // How much drift detected (degrees)


    /**
     * Initializes shooter subsystem: flywheel motors, turret encoder tracking, and ball flipper.
     */
    public Shooter() {
        this.robot = RobotHardware.getInstance();

        // Configure flywheel PIDF for velocity control
        com.qualcomm.robotcore.hardware.PIDFCoefficients pidCoefficients =
            new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                RobotConstants.Shooter.SHOOTER_P,
                RobotConstants.Shooter.SHOOTER_I,
                RobotConstants.Shooter.SHOOTER_D,
                RobotConstants.Shooter.SHOOTER_F
            );

        // Flywheel motor 1: velocity control mode
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        // Flywheel motor 2: velocity control mode (reversed to spin same direction)
        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidCoefficients);

        // Initialize turret PID timing
        lastTurretTime = System.nanoTime();

        // Initialize turret encoder cumulative tracking
        // Read initial position, apply calibration offset, and normalize to [-180, 180]
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;
        rawDegrees -= RobotConstants.Shooter.TURRET_ENCODER_OFFSET;  // Apply calibration
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;
        lastRawTurretPosition = rawDegrees;
        turretRotationCount = 0;
        cumulativeTurretPosition = rawDegrees;

        // Initialize drift detection
        lastRawForDrift = rawDegrees;
        expectedCumulative = rawDegrees;
        driftDetected = false;
        driftAmount = 0;

        // Start with ball flipper retracted
        robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_RETRACT);
    }


    /** Returns current flywheel motor power (0 to 1). */
    public double getFlywheelPower() {
        return robot.shooterMotor1.getPower();
    }

    /**
     * Calculates the turret's position in field coordinates (inches).
     *
     * PEDRO PATHING COORDINATE SYSTEM:
     *   - Origin (0,0) at bottom-left of field
     *   - +X = RIGHT, +Y = FORWARD/UP
     *   - Heading 0° = facing right (+X), 90° = facing forward (+Y)
     *   - Counter-clockwise rotation is positive
     *
     * TURRET OFFSET (robot-relative):
     *   - OFFSET_X = 4" to the RIGHT of robot center
     *   - OFFSET_Y = 1" FORWARD of robot center
     *
     * TRANSFORM MATH:
     *   - Robot's forward direction in field: (cos θ, sin θ)
     *   - Robot's right direction in field: (sin θ, -cos θ)
     *   - turretPos = robotPos + OFFSET_Y * forward + OFFSET_X * right
     *
     * @return [turretX, turretY] in field coordinates (inches)
     */
    private double[] getTurretFieldPosition() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // Apply 2D rotation to transform robot-relative offset to field coordinates
        double turretX = currentPose.getX(DistanceUnit.INCH) +
                        (RobotConstants.Shooter.TURRET_OFFSET_X * Math.sin(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.cos(robotHeading));
        double turretY = currentPose.getY(DistanceUnit.INCH) +
                        (-RobotConstants.Shooter.TURRET_OFFSET_X * Math.cos(robotHeading) +
                         RobotConstants.Shooter.TURRET_OFFSET_Y * Math.sin(robotHeading));

        return new double[]{turretX, turretY};
    }

    /** Returns straight-line distance from turret to goal (inches). */
    public double getDistanceToTarget() {
        double[] turretPos = getTurretFieldPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    /**
     * Looks up flywheel velocity for a given distance using linear interpolation.
     * @param distance Distance to goal in inches
     * @return Flywheel velocity in ticks per second
     */
    private double getVelocityFromDistance(double distance) {
        //Offset from 5,5 inch distance for shooting, fix goal eveentually lol
        distance -= Math.sqrt(4);
        // Return fixed velocity during Limelight scan mode
        if (RobotConstants.Limelight.manuallySlowedForScan) {
            return 2200.0;
        }

        // Clamp to table bounds
        if (distance <= RobotConstants.Shooter.VELOCITY_LOOKUP[0][0]) {
            return RobotConstants.Shooter.VELOCITY_LOOKUP[0][1];
        }
        if (distance >= RobotConstants.Shooter.VELOCITY_LOOKUP[RobotConstants.Shooter.VELOCITY_LOOKUP.length - 1][0]) {
            return RobotConstants.Shooter.VELOCITY_LOOKUP[RobotConstants.Shooter.VELOCITY_LOOKUP.length - 1][1];
        }

        // Linear interpolation between adjacent table entries
        for (int i = 0; i < RobotConstants.Shooter.VELOCITY_LOOKUP.length - 1; i++) {
            double dist1 = RobotConstants.Shooter.VELOCITY_LOOKUP[i][0];
            double dist2 = RobotConstants.Shooter.VELOCITY_LOOKUP[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                double vel1 = RobotConstants.Shooter.VELOCITY_LOOKUP[i][1];
                double vel2 = RobotConstants.Shooter.VELOCITY_LOOKUP[i + 1][1];
                double ratio = (distance - dist1) / (dist2 - dist1);
                return vel1 + (vel2 - vel1) * ratio;
            }
        }

        return 2200.0; // Fallback
    }

    /** Updates requiredVelocity based on current distance to goal. */
    private void updateVelocityFromDistance() {
        double distance = getDistanceToTarget();
        requiredVelocity = getVelocityFromDistance(distance);
    }


    /** Ball flipper state machine - extends then retracts to push ball into flywheel. */
    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return;

        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.shooterFlipper.setPosition(RobotConstants.Shooter.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                currentState = FlickState.Idle;
                break;
        }
    }

    /**
     * Sets turret target based on current mode:
     * - TagTracking mode: points at AprilTag scan position (72, 143)
     * - ShootingZone: tracks alliance goal
     * - IdleZone: centers turret
     */
    public void turretPeriodic() {
        if (limelightSubsystem != null &&
            limelightSubsystem.getCurrentMode() == RobotConstants.Enums.LimelightMode.TagTracking) {
            setTurretDegree(getDegreesToTagGoal());
        } else {
            switch (fieldState) {
                case IdleZone:
                    setTurretDegree(CENTER);
                    break;
                case ShootingZone:
                    setTurretDegree(getDegreesToGoal());
                    break;
            }
        }
    }

    /** Sets turret target angle in TURRET degrees (not servo degrees). */
    public void setTurretDegree(double degree) {
        setTurretAngle(degree);
    }

    /**
     * Calculates turret angle needed to point at alliance goal.
     *
     * COORDINATE SYSTEM (Pedro Pathing):
     *   - Origin (0,0) at bottom-left of field
     *   - +X = right, +Y = forward/up
     *   - Heading 0° = facing +X, 90° = facing +Y
     *   - CCW rotation is positive
     *
     * TURRET CONVENTION:
     *   - 0° = turret centered (forward)
     *   - Positive = turret pointing left (CCW)
     *   - Negative = turret pointing right (CW)
     *
     * @return Turret angle in degrees (robot-relative)
     */
    public double getDegreesToGoal() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        // Get turret field position (accounts for 4" right, 1" forward offset)
        double[] turretPos = getTurretFieldPosition();

        // Vector from turret to goal
        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        // Field angle to goal (standard math: 0° = +X, CCW positive)
        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative by subtracting robot heading
        double robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);
        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        // Convert to degrees and normalize to [-180, 180]
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // Apply fine-tune calibration offset
        turretAngleDeg += RobotConstants.Shooter.TURRET_TRACKING_OFFSET;

        // Store diagnostic data for debugging
        diagRobotX = currentPose.getX(DistanceUnit.INCH);
        diagRobotY = currentPose.getY(DistanceUnit.INCH);
        diagRobotHeading = Math.toDegrees(robotHeadingRad);
        diagTurretX = turretPos[0];
        diagTurretY = turretPos[1];
        diagGoalX = goalPosition.getX();
        diagGoalY = goalPosition.getY();
        diagDeltaX = deltaX;
        diagDeltaY = deltaY;
        diagAbsoluteAngle = Math.toDegrees(fieldAngleRad);
        diagRelativeAngle = turretAngleDeg;

        // Check if target exceeds hardware limits
        if (turretAngleDeg > RobotConstants.Shooter.TURRET_MAX_ANGLE) {
            targetOutOfRange = true;
            degreesOutOfRange = turretAngleDeg - RobotConstants.Shooter.TURRET_MAX_ANGLE;
        } else if (turretAngleDeg < RobotConstants.Shooter.TURRET_MIN_ANGLE) {
            targetOutOfRange = true;
            degreesOutOfRange = RobotConstants.Shooter.TURRET_MIN_ANGLE - turretAngleDeg;
        } else {
            targetOutOfRange = false;
            degreesOutOfRange = 0;
        }

        return turretAngleDeg;
    }

    /**
     * Normalizes angle to [-180, 180] range.
     * @param degrees Angle in degrees
     * @return Normalized angle in [-180, 180]
     */
    private double normalizeAngle(double degrees) {
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Calculates turret angle to point at AprilTag scanning position (72, 143).
     * Used during TagTracking mode to position Limelight camera.
     * @return Turret angle in degrees (robot-relative)
     */
    public double getDegreesToTagGoal() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double[] turretPos = getTurretFieldPosition();

        double deltaX = RobotConstants.Limelight.TAG_GOAL_X - turretPos[0];
        double deltaY = RobotConstants.Limelight.TAG_GOAL_Y - turretPos[1];

        double absoluteAngle = Math.atan2(deltaY, deltaX);
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double relativeAngle = Math.toDegrees(absoluteAngle - robotHeading);

        while (relativeAngle > 180) relativeAngle -= 360;
        while (relativeAngle < -180) relativeAngle += 360;

        relativeAngle += RobotConstants.Shooter.TURRET_TRACKING_OFFSET;

        return relativeAngle;
    }

    // ==================== TURRET CONTROL ====================

    /**
     * Reads turret encoder and returns cumulative position in SERVO degrees.
     *
     * CUMULATIVE TRACKING:
     *   - Encoder reads 0-360° (from voltage), normalized to [-180, 180]
     *   - Tracks boundary crossings when encoder wraps around ±180°
     *   - Cumulative position = raw + (rotationCount × 360)
     *   - This allows turret to rotate beyond one full revolution
     *
     * UNITS: Returns SERVO degrees (multiply by GEAR_RATIO from turret degrees)
     * RANGE: Approximately -360° to +270° servo (limited by hardware)
     */
    public double getTurretPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Apply encoder calibration offset so that "turret centered" = 0°
        // CRITICAL: This offset must be calibrated or all angles will be wrong!
        rawDegrees -= RobotConstants.Shooter.TURRET_ENCODER_OFFSET;

        // Normalize to [-180, 180] for boundary detection
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        // Detect boundary crossings
        double delta = rawDegrees - lastRawTurretPosition;
        if (delta < -180) {
            turretRotationCount++; // Crossed +180 → -180 (CW)
        } else if (delta > 180) {
            turretRotationCount--; // Crossed -180 → +180 (CCW)
        }

        lastRawTurretPosition = rawDegrees;
        cumulativeTurretPosition = rawDegrees + (turretRotationCount * 360);

        // Drift detection: track expected cumulative based on actual deltas
        double driftDelta = rawDegrees - lastRawForDrift;
        // Handle wraparound for drift tracking too
        if (driftDelta > 180) driftDelta -= 360;
        if (driftDelta < -180) driftDelta += 360;
        expectedCumulative += driftDelta;
        lastRawForDrift = rawDegrees;

        // Check for drift (difference between tracked cumulative and expected)
        driftAmount = Math.abs(cumulativeTurretPosition - expectedCumulative);
        // Flag drift if more than 30 degrees off (indicates missed boundary crossing)
        driftDetected = driftAmount > 30.0;

        return cumulativeTurretPosition;
    }

    /**
     * Sets turret target angle with hardware limit clamping.
     *
     * LIMITS: Turret can physically rotate from TURRET_MIN_ANGLE (left) to TURRET_MAX_ANGLE (right)
     * GEAR RATIO: 5:1 servo-to-turret, so servo range is (MIN × 5) to (MAX × 5)
     *
     * @param targetAngleTurret Target in TURRET degrees (will be clamped to [MIN, MAX])
     */
    public void setTurretAngle(double targetAngleTurret) {
        // Clamp to hardware limits (-45° left to +60° right)
        targetAngleTurret = Math.max(
            RobotConstants.Shooter.TURRET_MIN_ANGLE,
            Math.min(RobotConstants.Shooter.TURRET_MAX_ANGLE, targetAngleTurret)
        );

        // Convert to servo degrees
        double newTargetServo = targetAngleTurret * RobotConstants.Shooter.GEAR_RATIO;
        targetTurretAngle = newTargetServo;

        // Reset PID state when target changes significantly
        if (Math.abs(newTargetServo - lastTargetTurretAngle) > 0.5) {
            lastTargetTurretAngle = newTargetServo;
            turretIntegral = 0;
            lastTurretError = 0;
        }
    }

    /**
     * PID control loop for turret positioning. Called each periodic cycle.
     *
     * OPERATION:
     *   - Calculates error between target and current position (servo degrees)
     *   - Applies PID with anti-windup
     *   - Enforces hardware limits based on TURRET_MIN_ANGLE and TURRET_MAX_ANGLE
     *   - Reduces power near limits to prevent damage
     *   - Deadband stops motor when within ANGLE_RANGE of target
     */
    public void turretRotationUpdater() {
        double currentPosition = getTurretPosition();
        double error = targetTurretAngle - currentPosition;

        // Store error for diagnostics
        diagTurretError = error;

        // Hardware limits (servo degrees) - calculated from turret degree limits
        // Positive = right (CW), Negative = left (CCW)
        final double CW_LIMIT = RobotConstants.Shooter.TURRET_MAX_ANGLE * RobotConstants.Shooter.GEAR_RATIO;   // +60° turret × 5 = +300°
        final double CCW_LIMIT = RobotConstants.Shooter.TURRET_MIN_ANGLE * RobotConstants.Shooter.GEAR_RATIO; // -45° turret × 5 = -225°
        final double LIMIT_MARGIN = 30.0;

        boolean nearCWLimit = currentPosition > (CW_LIMIT - LIMIT_MARGIN);
        boolean nearCCWLimit = currentPosition < (CCW_LIMIT + LIMIT_MARGIN);

        // Stop if at limit and trying to go further
        if ((currentPosition >= CW_LIMIT && error > 0) ||
            (currentPosition <= CCW_LIMIT && error < 0)) {
            robot.turretServo.setPower(0);
            turretIntegral = 0;
            return;
        }

        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTurretTime) / 1e9;
        lastTurretTime = currentTime;
        if (dt > 1.0 || dt < 0.001) dt = 0.02;

        // PID calculation with anti-windup
        turretIntegral += error * dt;
        turretIntegral = Math.max(-50, Math.min(50, turretIntegral));
        double derivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        double power = (turretKP * error) + (turretKI * turretIntegral) + (turretKD * derivative);

        // Safety check
        if (!Double.isFinite(power)) {
            robot.turretServo.setPower(0);
            turretIntegral = 0;
            lastTurretError = 0;
            return;
        }

        // Power limiting
        double maxPower = (nearCWLimit || nearCCWLimit) ? 0.3 : 0.5;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Deadband - stop when close enough to target
        if (Math.abs(error) < RobotConstants.Shooter.ANGLE_RANGE) {
            power = 0;
            turretIntegral = 0;
        }

        robot.turretServo.setPower(power);
    }

    /** Returns current turret target in SERVO degrees. */
    public double getTargetTurretAngle() {
        return targetTurretAngle;
    }

    /** Returns current required flywheel velocity (ticks/sec). */
    public double getRequiredVelocity() {
        return requiredVelocity;
    }

    /** Triggers ball flipper to fire a ball (if not already firing). */
    public void triggerShot() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }

    // ==================== STATE QUERIES ====================

    /** Returns current flipper state. */
    public FlickState getCurrentState() {
        return currentState;
    }

    /** Toggles Limelight enabled/disabled. */
    public void toggleLimelightEnabled() {
        RobotConstants.Limelight.isLimelightDisabled = !RobotConstants.Limelight.isLimelightDisabled;
    }

    /** Sets Limelight subsystem reference for mode switching. */
    public void setLimelightSubsystem(subsystems.Limelight limelight) {
        this.limelightSubsystem = limelight;
    }

    /** Updates turret PID gains (for live tuning). */
    public void setTurretPID(double kP, double kI, double kD) {
        this.turretKP = kP;
        this.turretKI = kI;
        this.turretKD = kD;
    }

    /** Enables/disables manual velocity mode (bypasses distance-based velocity). */
    public void setManualVelocityMode(boolean enabled) {
        this.manualVelocityMode = enabled;
    }

    /** Sets manual velocity target (only used when manualVelocityMode is true). */
    public void setManualVelocity(double velocity) {
        this.manualVelocity = velocity;
    }

    /** Returns current manual velocity setting. */
    public double getManualVelocity() {
        return manualVelocity;
    }

    /** Returns true if manual velocity mode is enabled. */
    public boolean isManualVelocityMode() {
        return manualVelocityMode;
    }

    // ==================== DIAGNOSTIC TELEMETRY ====================

    /**
     * Adds comprehensive turret tracking diagnostic data to telemetry.
     * Call this from TeleOp to debug turret aiming issues.
     *
     * Shows:
     *   - Robot position (X, Y, Heading)
     *   - Turret field position (calculated from offset)
     *   - Goal position
     *   - Delta to goal
     *   - Angle calculations (absolute, relative)
     *   - Turret target vs actual position
     *   - PID error
     *
     * @param telemetry The telemetry object from the OpMode
     */
    public void addDiagnosticTelemetry(Telemetry telemetry) {
        // Show out-of-range warning prominently at top
        if (targetOutOfRange) {
            telemetry.addLine("!! TARGET OUT OF RANGE !!");
            telemetry.addData("Degrees beyond limit", String.format("%.1f°", degreesOutOfRange));
            telemetry.addData("Rotate robot", degreesOutOfRange > 0 ? "RIGHT" : "LEFT");
        }

        telemetry.addLine("=== TURRET DIAGNOSTICS ===");

        // Robot position
        telemetry.addData("Robot X (in)", String.format("%.2f", diagRobotX));
        telemetry.addData("Robot Y (in)", String.format("%.2f", diagRobotY));
        telemetry.addData("Robot Heading (deg)", String.format("%.1f", diagRobotHeading));

        telemetry.addLine("--- Turret Position ---");
        telemetry.addData("Turret Field X (in)", String.format("%.2f", diagTurretX));
        telemetry.addData("Turret Field Y (in)", String.format("%.2f", diagTurretY));

        telemetry.addLine("--- Goal ---");
        telemetry.addData("Goal X (in)", String.format("%.1f", diagGoalX));
        telemetry.addData("Goal Y (in)", String.format("%.1f", diagGoalY));
        telemetry.addData("Delta X (in)", String.format("%.2f", diagDeltaX));
        telemetry.addData("Delta Y (in)", String.format("%.2f", diagDeltaY));
        telemetry.addData("Distance (in)", String.format("%.1f", getDistanceToTarget()));

        telemetry.addLine("--- Angle Calculation ---");
        telemetry.addData("Absolute Angle (deg)", String.format("%.1f", diagAbsoluteAngle));
        telemetry.addData("Relative Angle (deg)", String.format("%.1f", diagRelativeAngle));
        telemetry.addData("In Range", targetOutOfRange ? "NO" : "YES");

        telemetry.addLine("--- Turret Control ---");
        double turretTargetDeg = targetTurretAngle / RobotConstants.Shooter.GEAR_RATIO;
        double turretActualDeg = getTurretPosition() / RobotConstants.Shooter.GEAR_RATIO;
        double turretErrorDeg = diagTurretError / RobotConstants.Shooter.GEAR_RATIO;
        telemetry.addData("Target (turret deg)", String.format("%.1f", turretTargetDeg));
        telemetry.addData("Actual (turret deg)", String.format("%.1f", turretActualDeg));
        telemetry.addData("PID Error (turret deg)", String.format("%.2f", turretErrorDeg));

        // Key diagnostic: compare CALCULATED target vs WHERE turret actually is
        // If "Calculated Aim" != "Actual Turret Pos", PID isn't converging
        // If they match but shots miss, the CALCULATION is wrong
        telemetry.addLine("--- KEY DIAGNOSTIC ---");
        telemetry.addData("Calculated Aim Angle", String.format("%.1f° (from getDegreesToGoal)", diagRelativeAngle));
        telemetry.addData("Turret Physical Pos", String.format("%.1f° (from encoder)", turretActualDeg));
        telemetry.addData("Difference", String.format("%.1f° (should be ~0 when settled)", diagRelativeAngle - turretActualDeg));
        telemetry.addData("Gear Ratio", String.format("%.2f", RobotConstants.Shooter.GEAR_RATIO));

        telemetry.addLine("--- Status ---");
        telemetry.addData("Field State", fieldState.toString());
        telemetry.addData("Flywheel Velocity", String.format("%.0f", requiredVelocity));

        // Drift detection
        if (driftDetected) {
            telemetry.addLine("!! POSITION DRIFT DETECTED !!");
            telemetry.addData("Drift amount", String.format("%.1f° - consider resetting", driftAmount));
        }
        telemetry.addData("Rotation Count", turretRotationCount);
        telemetry.addData("Cumulative Pos", String.format("%.1f°", cumulativeTurretPosition));
        telemetry.addData("Expected Pos", String.format("%.1f°", expectedCumulative));

        // Encoder calibration data
        telemetry.addLine("--- Encoder Data ---");
        double rawVoltage = robot.turretEncoder.getVoltage();
        double rawDegreesUncalibrated = (rawVoltage / 3.3) * 360.0;
        telemetry.addData("Raw Voltage", String.format("%.3f V", rawVoltage));
        telemetry.addData("Raw Degrees (uncal)", String.format("%.1f°", rawDegreesUncalibrated));
        telemetry.addData("Encoder Offset", String.format("%.1f°", RobotConstants.Shooter.TURRET_ENCODER_OFFSET));

        // Physical verification data
        telemetry.addLine("--- VERIFY THESE VALUES ---");
        telemetry.addData("Goal Position", String.format("(%.0f, %.0f)", diagGoalX, diagGoalY));
    }

    /**
     * Returns a summary string of turret tracking status (for compact display).
     * Format: "Turret: target°→actual° (err°) [zone]" or "!! OUT OF RANGE !!" if target unreachable
     */
    public String getDiagnosticSummary() {
        if (targetOutOfRange) {
            return String.format("!! OUT OF RANGE by %.1f° - rotate robot !!", degreesOutOfRange);
        }
        double turretTargetDeg = targetTurretAngle / RobotConstants.Shooter.GEAR_RATIO;
        double turretActualDeg = getTurretPosition() / RobotConstants.Shooter.GEAR_RATIO;
        double errorDeg = diagTurretError / RobotConstants.Shooter.GEAR_RATIO;
        return String.format("Turret: %.1f°→%.1f° (err:%.1f°) [%s]",
                turretTargetDeg, turretActualDeg, errorDeg, fieldState.toString());
    }

    /** Returns true if the target angle exceeds hardware limits (-60° to +45°). */
    public boolean isTargetOutOfRange() {
        return targetOutOfRange;
    }

    /** Returns how many degrees beyond the limit the target is (0 if in range). */
    public double getDegreesOutOfRange() {
        return degreesOutOfRange;
    }

    /** Returns true if cumulative position tracking has drifted significantly. */
    public boolean isDriftDetected() {
        return driftDetected;
    }

    /** Returns the amount of detected drift in degrees. */
    public double getDriftAmount() {
        return driftAmount;
    }

    /**
     * Resets the drift detection tracking.
     * Call this when turret is at a known position (e.g., after centering).
     */
    public void resetDriftTracking() {
        expectedCumulative = cumulativeTurretPosition;
        driftDetected = false;
        driftAmount = 0;
    }

    // ==================== ZONE DETECTION ====================

    /**
     * Updates fieldState based on robot position on the field.
     *
     * Checks all four corners of the robot against the FieldMap bitmap.
     * If ANY corner is in a shooting zone ('S'), sets fieldState to ShootingZone.
     * Otherwise sets to IdleZone.
     *
     * Corner positions are transformed from robot frame to field frame using
     * the standard 2D rotation matrix.
     */
    public void updateFieldState() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double centerX = currentPose.getX(DistanceUnit.INCH);
        double centerY = currentPose.getY(DistanceUnit.INCH);
        double heading = currentPose.getHeading(AngleUnit.RADIANS);
        double halfSize = RobotConstants.Robot.HALF_SIZE;

        // Robot corners in robot-relative coordinates [forward, right]
        // forward = along robot's facing direction
        // right = 90° clockwise from forward
        double[][] corners = {
            { halfSize,  halfSize},  // Front-right
            { halfSize, -halfSize},  // Front-left
            {-halfSize,  halfSize},  // Back-right
            {-halfSize, -halfSize}   // Back-left
        };

        boolean inShootingZone = false;
        for (double[] corner : corners) {
            // Transform to field coordinates using Pedro Pathing convention:
            // Forward direction in field: (cos θ, sin θ)
            // Right direction in field: (sin θ, -cos θ)
            // cornerPos = centerPos + forward * corner[0] + right * corner[1]
            double cornerX = centerX + (corner[0] * Math.cos(heading) + corner[1] * Math.sin(heading));
            double cornerY = centerY + (corner[0] * Math.sin(heading) - corner[1] * Math.cos(heading));

            if (FieldMap.getPosition(cornerX, cornerY) == 'S') {
                inShootingZone = true;
                break;
            }
        }

        fieldState = inShootingZone ? FieldState.ShootingZone : FieldState.IdleZone;
    }

    /** Returns current field zone state. */
    public FieldState getFieldState() {
        return fieldState;
    }

    // ==================== MAIN LOOP ====================

    /**
     * Main periodic update - called every loop cycle.
     *
     * Executes in order:
     *   1. Update flywheel velocity (from distance or manual)
     *   2. Run ball flipper state machine
     *   3. Update field zone state from robot position
     *   4. Set turret target based on mode and zone
     *   5. Run turret PID control
     */
    @Override
    public void periodic() {
        // Set flywheel velocity
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;
        if (!manualVelocityMode) {
            updateVelocityFromDistance();
            targetVelocity = requiredVelocity;
        }
        robot.shooterMotor1.setVelocity(targetVelocity);
        robot.shooterMotor2.setVelocity(targetVelocity);

        // Run subsystem updates
        flipperStateMachinePeriodic();
        updateFieldState();
        turretPeriodic();
        turretRotationUpdater();
    }
}