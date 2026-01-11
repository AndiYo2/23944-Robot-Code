package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Constants.EnumConstants;
import Constants.EnumConstants.FieldState;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.TurretConstants;

import static Constants.TurretConstants.CENTER;

public class Turret implements Subsystem {
    // Hardware reference
    RobotHardware robot;

    // Turret PID state variables (all angles in SERVO degrees, not turret degrees)
    private double targetTurretAngle = 0.0;
    private double lastTargetTurretAngle = 0.0;
    private double lastTurretError = 0;
    private double turretIntegral = 0;
    private long lastTurretTime = 0;

    // Cumulative position tracking for turret encoder
    // Needed because turret can rotate beyond 360 (servo range: -360 to +270)
    private double lastRawTurretPosition = 0;
    private double cumulativeTurretPosition = 0;
    private int turretRotationCount = 0;

    // Limelight reference for switching between goal tracking and tag scanning modes
    private subsystems.Limelight limelightSubsystem;

    // Out-of-range tracking - set when target angle exceeds hardware limits
    private boolean targetOutOfRange = false;
    private double degreesOutOfRange = 0;

    // Drift detection - tracks if cumulative position seems to have drifted
    private double lastRawForDrift = 0;
    private double expectedCumulative = 0;
    private boolean driftDetected = false;
    private double driftAmount = 0;


    public Turret() {
        this.robot = RobotHardware.getInstance();

        // Initialize turret PID timing
        lastTurretTime = System.nanoTime();

        // Initialize turret encoder cumulative tracking
        // Read initial position, apply calibration offset, and normalize to [-180, 180]
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        rawDegrees -= TurretConstants.TURRET_ENCODER_OFFSET;  // Apply calibration
        while (rawDegrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) rawDegrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (rawDegrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) rawDegrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        lastRawTurretPosition = rawDegrees;
        turretRotationCount = 0;
        cumulativeTurretPosition = rawDegrees;

        // Initialize drift detection
        lastRawForDrift = rawDegrees;
        expectedCumulative = rawDegrees;
        driftDetected = false;
        driftAmount = 0;
    }

    public double[] getTurretFieldPosition() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);

        // Apply 2D rotation to transform robot-relative offset to field coordinates
        double turretX = currentPose.getX(DistanceUnit.INCH) +
                        (TurretConstants.TURRET_OFFSET_X * Math.sin(robotHeading) +
                         TurretConstants.TURRET_OFFSET_Y * Math.cos(robotHeading));
        double turretY = currentPose.getY(DistanceUnit.INCH) +
                        (-TurretConstants.TURRET_OFFSET_X * Math.cos(robotHeading) +
                         TurretConstants.TURRET_OFFSET_Y * Math.sin(robotHeading));

        return new double[]{turretX, turretY};
    }

    public void turretPeriodic(FieldState fieldState) {
        if (limelightSubsystem != null &&
            limelightSubsystem.getCurrentMode() == EnumConstants.LimelightMode.TagTracking) {
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

    public void setTurretDegree(double degree) {
        setTurretAngle(degree);
    }

    public double getDegreesToGoal() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        Pose goalPosition = FieldMap.getGoalPosition();

        // Get turret field position (accounts for offset from robot center)
        double[] turretPos = getTurretFieldPosition();

        // Vector from turret to goal
        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        // Field angle to goal (standard math: 0 = +X, CCW positive)
        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative by subtracting robot heading
        double robotHeadingRad = currentPose.getHeading(AngleUnit.RADIANS);
        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        // Convert to degrees and normalize to [-180, 180]
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // Apply fine-tune calibration offset
        turretAngleDeg += TurretConstants.TURRET_TRACKING_OFFSET;

        // Check if target exceeds hardware limits
        if (turretAngleDeg > TurretConstants.TURRET_MAX_ANGLE) {
            targetOutOfRange = true;
            degreesOutOfRange = turretAngleDeg - TurretConstants.TURRET_MAX_ANGLE;
        } else if (turretAngleDeg < TurretConstants.TURRET_MIN_ANGLE) {
            targetOutOfRange = true;
            degreesOutOfRange = TurretConstants.TURRET_MIN_ANGLE - turretAngleDeg;
        } else {
            targetOutOfRange = false;
            degreesOutOfRange = 0;
        }

        return turretAngleDeg;
    }

    private double normalizeAngle(double degrees) {
        while (degrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) degrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (degrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) degrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        return degrees;
    }

    public double getDegreesToTagGoal() {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double[] turretPos = getTurretFieldPosition();

        double deltaX = LimelightConstants.TAG_GOAL_X - turretPos[0];
        double deltaY = LimelightConstants.TAG_GOAL_Y - turretPos[1];

        double absoluteAngle = Math.atan2(deltaY, deltaX);
        double robotHeading = currentPose.getHeading(AngleUnit.RADIANS);
        double relativeAngle = Math.toDegrees(absoluteAngle - robotHeading);

        while (relativeAngle > RobotConstants.Encoder.ANGLE_UPPER_BOUND) relativeAngle -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (relativeAngle < RobotConstants.Encoder.ANGLE_LOWER_BOUND) relativeAngle += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        relativeAngle += TurretConstants.TURRET_TRACKING_OFFSET;

        return relativeAngle;
    }

    public double getTurretPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Apply encoder calibration offset so that "turret centered" = 0
        rawDegrees -= TurretConstants.TURRET_ENCODER_OFFSET;

        // Normalize to [-180, 180] for boundary detection
        while (rawDegrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) rawDegrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (rawDegrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) rawDegrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Detect boundary crossings
        double delta = rawDegrees - lastRawTurretPosition;
        if (delta < RobotConstants.Encoder.ANGLE_LOWER_BOUND) {
            turretRotationCount++; // Crossed +180 -> -180 (CW)
        } else if (delta > RobotConstants.Encoder.ANGLE_UPPER_BOUND) {
            turretRotationCount--; // Crossed -180 -> +180 (CCW)
        }

        lastRawTurretPosition = rawDegrees;
        cumulativeTurretPosition = rawDegrees + (turretRotationCount * RobotConstants.Encoder.FULL_ROTATION_DEGREES);

        // Drift detection: track expected cumulative based on actual deltas
        double driftDelta = rawDegrees - lastRawForDrift;
        // Handle wraparound for drift tracking too
        if (driftDelta > RobotConstants.Encoder.ANGLE_UPPER_BOUND) driftDelta -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        if (driftDelta < RobotConstants.Encoder.ANGLE_LOWER_BOUND) driftDelta += RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        expectedCumulative += driftDelta;
        lastRawForDrift = rawDegrees;

        // Check for drift (difference between tracked cumulative and expected)
        driftAmount = Math.abs(cumulativeTurretPosition - expectedCumulative);
        // Flag drift if more than threshold degrees off (indicates missed boundary crossing)
        driftDetected = driftAmount > TurretConstants.TURRET_DRIFT_THRESHOLD;

        return cumulativeTurretPosition;
    }

    public void setTurretAngle(double targetAngleTurret) {
        // Clamp to hardware limits (-45 left to +60 right)
        targetAngleTurret = Math.max(
            TurretConstants.TURRET_MIN_ANGLE,
            Math.min(TurretConstants.TURRET_MAX_ANGLE, targetAngleTurret)
        );

        // Convert to servo degrees
        double newTargetServo = targetAngleTurret * TurretConstants.GEAR_RATIO;
        targetTurretAngle = newTargetServo;

        // Reset PID state when target changes significantly
        if (Math.abs(newTargetServo - lastTargetTurretAngle) > TurretConstants.TURRET_TARGET_CHANGE_THRESHOLD) {
            lastTargetTurretAngle = newTargetServo;
            turretIntegral = 0;
            lastTurretError = 0;
        }
    }

    public void turretRotationUpdater() {
        double currentPosition = getTurretPosition();
        double error = targetTurretAngle - currentPosition;

        // Hardware limits (servo degrees) - calculated from turret degree limits
        // Positive = right (CW), Negative = left (CCW)
        final double CW_LIMIT = TurretConstants.TURRET_MAX_ANGLE * TurretConstants.GEAR_RATIO;
        final double CCW_LIMIT = TurretConstants.TURRET_MIN_ANGLE * TurretConstants.GEAR_RATIO;
        final double LIMIT_MARGIN = TurretConstants.TURRET_LIMIT_MARGIN;

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
        if (dt > RobotConstants.PID.DT_MAX || dt < RobotConstants.PID.DT_MIN) dt = RobotConstants.PID.DT_DEFAULT;

        // PID calculation with anti-windup
        turretIntegral += error * dt;
        turretIntegral = Math.max(RobotConstants.PID.INTEGRAL_CLAMP_MIN, Math.min(RobotConstants.PID.INTEGRAL_CLAMP_MAX, turretIntegral));
        double derivative = (error - lastTurretError) / dt;
        lastTurretError = error;

        // Read PID gains directly from constants
        double power = (TurretConstants.TURRET_PID.p * error)
                     + (TurretConstants.TURRET_PID.i * turretIntegral)
                     + (TurretConstants.TURRET_PID.d * derivative);

        // Safety check
        if (!Double.isFinite(power)) {
            robot.turretServo.setPower(0);
            turretIntegral = 0;
            lastTurretError = 0;
            return;
        }

        // Power limiting
        double maxPower = (nearCWLimit || nearCCWLimit) ? TurretConstants.TURRET_POWER_LIMIT_NEAR_EDGE : TurretConstants.TURRET_POWER_LIMIT_NORMAL;
        power = Math.max(-maxPower, Math.min(maxPower, power));

        // Deadband - stop when close enough to target
        if (Math.abs(error) < TurretConstants.ANGLE_RANGE) {
            power = 0;
            turretIntegral = 0;
        }

        robot.turretServo.setPower(power);
    }

    public double getTargetTurretAngle() {
        return targetTurretAngle;
    }

    public void setLimelightSubsystem(subsystems.Limelight limelight) {
        this.limelightSubsystem = limelight;
    }

    @Override
    public void periodic() {
        turretRotationUpdater();
    }
}
