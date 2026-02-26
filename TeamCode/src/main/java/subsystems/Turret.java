package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;

import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.RobotConstants;
import Constants.ShooterConstants;
import utility.RobotHardware;
import Constants.TurretConstants;

import static Constants.TurretConstants.CENTER;

/**
 * Turret subsystem using position-controlled servo.
 *
 * Servo position 0.5 = turret center (0 degrees)
 * Conversion: servoPosition = 0.5 + (turretDegrees * GEAR_RATIO / SERVO_DEGREES_PER_UNIT)
 */
public class Turret extends SubsystemBase {
    RobotHardware robot;

    private double currentTargetDegrees = 0.0;

    private boolean targetOutOfRange = false;
    private double degreesOutOfRange = 0;

    private double smoothedTargetDegrees = 0.0;
    private boolean smoothingInitialized = false;
    private double rawTargetDegrees = 0.0;

    // Shooter reference for lead compensation (future pose)
    private Shooter shooter;

    // Cached degrees-to-goal from periodic() — avoids duplicate calculation in telemetry
    private double lastDegreesToGoal = 0.0;

    // Servo dirty flag — only write when position changes by more than epsilon
    private double lastServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

    // Pre-allocated array for getTurretFieldPosition() to avoid GC pressure
    private final double[] turretFieldPos = new double[2];

    // Pre-computed turret offset in polar form (magnitude and angle)
    // Avoids recomputing sin/cos of offset every call — these never change at runtime
    private static final double TURRET_OFFSET_MAG = Math.sqrt(
            TurretConstants.TURRET_OFFSET_X * TurretConstants.TURRET_OFFSET_X +
            TurretConstants.TURRET_OFFSET_Y * TurretConstants.TURRET_OFFSET_Y);
    private static final double TURRET_OFFSET_ANGLE = Math.atan2(
            TurretConstants.TURRET_OFFSET_Y, TurretConstants.TURRET_OFFSET_X);

    public Turret() {
        this.robot = RobotHardware.getInstance();

        currentTargetDegrees = CENTER;
    }

    public void initServoPositions() {
        applyServoPosition(CENTER);
    }

    /**
     * Convert turret degrees to servo position.
     * Servo position 0.5 = turret center (0 degrees)
     *
     * @param turretDegrees Target angle in turret degrees
     * @return Servo position (0 to 1)
     */
    private double turretDegreesToServoPosition(double turretDegrees) {
        double position = TurretConstants.SERVO_CENTER_POSITION +
                (turretDegrees * TurretConstants.GEAR_RATIO / TurretConstants.SERVO_DEGREES_PER_UNIT);

        position = Math.max(RobotConstants.Robot.MIN_SERVO_SAFE_POSITION, Math.min(1.0, position));
        return position;
    }


    /**
     * Apply the current target position to the servo, gated behind dirty flag.
     */
    private void applyServoPosition(double turretDegrees) {
        double servoPosition = turretDegreesToServoPosition(turretDegrees);
        if (Math.abs(servoPosition - lastServoPosition) > SERVO_EPSILON) {
            robot.turretServo.setPosition(servoPosition);
            lastServoPosition = servoPosition;
        }
    }

    public void setShooter(Shooter shooter) {
        this.shooter = shooter;
    }

    /**
     * Compute turret field position from cached robot pose using pre-computed polar offset.
     * Returns pre-allocated array — do NOT store the reference across calls.
     */
    public double[] getTurretFieldPosition() {
        double robotHeading = robot.cachedHeading;

        double combinedAngle = robotHeading + TURRET_OFFSET_ANGLE;
        turretFieldPos[0] = robot.cachedPoseX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        turretFieldPos[1] = robot.cachedPoseY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);

        return turretFieldPos;
    }

    public void setTurretDegree(double degree) {
        setTurretAngle(degree);
    }

    public double getDegreesToGoal() {
        Pose goalPosition = FieldMap.getGoalPosition();

        // Get turret field position (accounts for offset from robot center)
        double[] turretPos = getTurretFieldPosition();

        // Vector from turret to goal
        double deltaX = goalPosition.getX() - turretPos[0];
        double deltaY = goalPosition.getY() - turretPos[1];

        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative by subtracting robot heading
        double robotHeadingRad = robot.cachedHeading;
        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        // Convert to degrees and normalize to [-180, 180]
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        turretAngleDeg += (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? TurretConstants.BLUE_TURRET_TRACKING_OFFSET
                : TurretConstants.RED_TURRET_TRACKING_OFFSET;

        if (turretAngleDeg > TurretConstants.HARD_STOP_CW) {
            targetOutOfRange = true;
            degreesOutOfRange = turretAngleDeg - TurretConstants.HARD_STOP_CW;
        } else if (turretAngleDeg < TurretConstants.HARD_STOP_CCW) {
            targetOutOfRange = true;
            degreesOutOfRange = TurretConstants.HARD_STOP_CCW - turretAngleDeg;
        } else {
            targetOutOfRange = false;
            degreesOutOfRange = 0;
        }

        return turretAngleDeg;
    }

    /**
     * Calculate the turret angle needed to aim at the goal from a hypothetical robot position.
     * Useful for pre-aiming the turret before arriving at a position.
     *
     * @param robotX hypothetical robot X position (inches)
     * @param robotY hypothetical robot Y position (inches)
     * @param robotHeadingRad hypothetical robot heading (radians)
     * @return turret angle in degrees
     */
    public double getDegreesToGoalFromPosition(double robotX, double robotY, double robotHeadingRad) {
        Pose goalPosition = FieldMap.getGoalPosition();

        // Calculate turret field position using pre-computed polar offset
        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        double turretX = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        double turretY = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);

        // Vector from turret to goal
        double deltaX = goalPosition.getX() - turretX;
        double deltaY = goalPosition.getY() - turretY;

        // Field angle to goal
        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        // Convert to robot-relative
        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        // Convert to degrees and normalize
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        // Apply calibration offset (alliance-specific)
        turretAngleDeg += (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? TurretConstants.BLUE_TURRET_TRACKING_OFFSET
                : TurretConstants.RED_TURRET_TRACKING_OFFSET;

        if (turretAngleDeg > TurretConstants.HARD_STOP_CW) {
            targetOutOfRange = true;
            degreesOutOfRange = turretAngleDeg - TurretConstants.HARD_STOP_CW;
        } else if (turretAngleDeg < TurretConstants.HARD_STOP_CCW) {
            targetOutOfRange = true;
            degreesOutOfRange = TurretConstants.HARD_STOP_CCW - turretAngleDeg;
        } else {
            targetOutOfRange = false;
            degreesOutOfRange = 0;
        }

        return turretAngleDeg;
    }

    /**
     * Get turret angle using lead compensation from the Shooter's predicted future pose.
     * Falls back to standard getDegreesToGoal() if shoot-while-moving is disabled
     * or if Shooter reference is unavailable.
     */
    public double getDegreesToGoalLeadAdjusted() {
        if (!ShooterConstants.SHOOT_WHILE_MOVING_ENABLED || shooter == null) {
            return getDegreesToGoal();
        }
        double[] future = shooter.getFuturePose();
        return getDegreesToGoalFromPosition(future[0], future[1], future[2]);
    }

    /**
     * Normalizes angle to [-180, 180] range using modulo arithmetic (O(1)).
     */
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        else if (degrees < -180) degrees += 360;
        return degrees;
    }

    /**
     * Set the turret to a target angle.
     * EMA smoothing is applied for fluid motion; servo is written every frame.
     *
     * @param targetAngleTurret Target angle in turret degrees
     */
    public void setTurretAngle(double targetAngleTurret) {
        targetAngleTurret = Math.max(
            TurretConstants.HARD_STOP_CCW,
            Math.min(TurretConstants.HARD_STOP_CW, targetAngleTurret)
        );

        rawTargetDegrees = targetAngleTurret;

        // Apply EMA smoothing to reduce high-frequency oscillations
        if (!smoothingInitialized) {
            smoothedTargetDegrees = targetAngleTurret;
            smoothingInitialized = true;
        } else {
            smoothedTargetDegrees = TurretConstants.SMOOTHING_ALPHA * targetAngleTurret
                                  + (1.0 - TurretConstants.SMOOTHING_ALPHA) * smoothedTargetDegrees;
        }

        // Always update — no deadband gating, so servo tracks continuously
        currentTargetDegrees = smoothedTargetDegrees;
    }


    public double getTargetTurretAngle() {
        return currentTargetDegrees;
    }

    public boolean isTargetOutOfRange() {
        return targetOutOfRange;
    }

    public double getRawTargetDegrees() {
        return rawTargetDegrees;
    }

    /** Get the cached degrees-to-goal computed in the last periodic() call. */
    public double getLastDegreesToGoal() {
        return lastDegreesToGoal;
    }

    @Override
    public void periodic() {
        lastDegreesToGoal = getDegreesToGoalLeadAdjusted();
        setTurretDegree(lastDegreesToGoal);
        applyServoPosition(currentTargetDegrees);
    }
}
