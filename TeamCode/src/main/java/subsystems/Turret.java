package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import Constants.EnumConstants.FieldState;
import Constants.FieldMap;
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
    // Hardware reference
    RobotHardware robot;

    // Current target angle (turret degrees)
    private double currentTargetDegrees = 0.0;

    // Last set angle for change threshold comparison
    private double lastSetDegrees = 0.0;

    // Out-of-range tracking - set when target angle exceeds hardware limits
    private boolean targetOutOfRange = false;
    private double degreesOutOfRange = 0;

    public Turret() {
        this.robot = RobotHardware.getInstance();

        // Initialize turret to center position
        currentTargetDegrees = CENTER;
        lastSetDegrees = CENTER;
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

        // Clamp to valid servo range and never use exactly 0
        position = Math.max(TurretConstants.MIN_SERVO_POSITION, Math.min(1.0, position));
        return position;
    }

    /**
     * Convert servo position to turret degrees.
     *
     * @param servoPosition Servo position (0 to 1)
     * @return Turret angle in degrees
     */
    private double servoPositionToTurretDegrees(double servoPosition) {
        return (servoPosition - TurretConstants.SERVO_CENTER_POSITION) *
                TurretConstants.SERVO_DEGREES_PER_UNIT / TurretConstants.GEAR_RATIO;
    }

    /**
     * Apply the current target position to the servo.
     */
    private void applyServoPosition(double turretDegrees) {
        double servoPosition = turretDegreesToServoPosition(turretDegrees);
        robot.turretServo.setPosition(servoPosition);
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
        // Always track to goal regardless of field state
        setTurretDegree(getDegreesToGoal());
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

        // Calculate turret field position from hypothetical robot position
        double turretX = robotX +
                (TurretConstants.TURRET_OFFSET_X * Math.sin(robotHeadingRad) +
                 TurretConstants.TURRET_OFFSET_Y * Math.cos(robotHeadingRad));
        double turretY = robotY +
                (-TurretConstants.TURRET_OFFSET_X * Math.cos(robotHeadingRad) +
                 TurretConstants.TURRET_OFFSET_Y * Math.sin(robotHeadingRad));

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

        // Apply calibration offset
        turretAngleDeg += TurretConstants.TURRET_TRACKING_OFFSET;

        return turretAngleDeg;
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
     * Only updates if the change exceeds MIN_CHANGE_THRESHOLD to prevent noise.
     *
     * @param targetAngleTurret Target angle in turret degrees
     */
    public void setTurretAngle(double targetAngleTurret) {
        // Clamp to hard stop limits
        targetAngleTurret = Math.max(
            TurretConstants.HARD_STOP_CCW,
            Math.min(TurretConstants.HARD_STOP_CW, targetAngleTurret)
        );

        // Check if change exceeds threshold
        if (Math.abs(targetAngleTurret - lastSetDegrees) >= TurretConstants.MIN_CHANGE_THRESHOLD) {
            currentTargetDegrees = targetAngleTurret;
            lastSetDegrees = targetAngleTurret;
        }
    }

    /**
     * Get the current target angle in turret degrees.
     */
    public double getTargetTurretAngle() {
        return currentTargetDegrees;
    }

    /**
     * Get the current servo position (0 to 1).
     */
    public double getServoPosition() {
        return turretDegreesToServoPosition(currentTargetDegrees);
    }

    /**
     * Check if target is out of range.
     */
    public boolean isTargetOutOfRange() {
        return targetOutOfRange;
    }

    /**
     * Get how many degrees the target is out of range.
     */
    public double getDegreesOutOfRange() {
        return degreesOutOfRange;
    }

    @Override
    public void periodic() {
        // Apply current target position to servo
        applyServoPosition(currentTargetDegrees);
    }
}
