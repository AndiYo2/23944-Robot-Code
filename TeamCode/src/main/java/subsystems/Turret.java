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

    private Shooter shooter;

    private double lastDegreesToGoal = 0.0;

    private double lastServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

    // Pre-allocated array for getTurretFieldPosition() to avoid GC pressure
    private final double[] turretFieldPos = new double[2];

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

    private double getRedTrackingOffset(double robotY) {
        return (robotY <= TurretConstants.TURRET_OFFSET_Y_THRESHOLD)
                ? TurretConstants.RED_BACK_TURRET_TRACKING_OFFSET
                : TurretConstants.RED_FRONT_TURRET_TRACKING_OFFSET;
    }

    private double getBlueTrackingOffset(double robotY) {
        return (robotY <= TurretConstants.TURRET_OFFSET_Y_THRESHOLD)
                ? TurretConstants.BLUE_BACK_TURRET_TRACKING_OFFSET
                : TurretConstants.BLUE_FRONT_TURRET_TRACKING_OFFSET;
    }

    public double[] getTurretFieldPosition() {
        double robotHeading = robot.cachedHeading;

        turretFieldPos[0] = robot.cachedPoseX + TURRET_OFFSET_MAG * Math.sin(robotHeading);
        turretFieldPos[1] = robot.cachedPoseY - TURRET_OFFSET_MAG * Math.cos(robotHeading);

        return turretFieldPos;
    }

    public void setTurretDegree(double degree) {
        setTurretAngle(degree);
    }

    public double getDegreesToGoal() {
        Pose goalPosition = FieldMap.getGoalPosition();

        double[] turretPos = getTurretFieldPosition();

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
                ? getBlueTrackingOffset(robot.cachedPoseY)
                : getRedTrackingOffset(robot.cachedPoseY);

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
     * Used for pre-aiming the turret before arriving at a position.
     *
     * @param robotX hypothetical robot X position (inches)
     * @param robotY hypothetical robot Y position (inches)
     * @param robotHeadingRad hypothetical robot heading (radians)
     * @return turret angle in degrees
     */
    public double getDegreesToGoalFromPosition(double robotX, double robotY, double robotHeadingRad) {
        Pose goalPosition = FieldMap.getGoalPosition();

        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        double turretX = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        double turretY = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);

        double deltaX = goalPosition.getX() - turretX;
        double deltaY = goalPosition.getY() - turretY;

        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        // Convert to degrees and normalize
        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        turretAngleDeg += (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? getBlueTrackingOffset(robotY)
                : getRedTrackingOffset(robotY);

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
     * Shooting_While_Moving turret aiming.
     * Computes turret angle from the kinematically-predicted future pose
     * (includes acceleration, not just velocity).
     */
    public double getDegreesToGoalShootingWhileMoving() {
        if (!ShooterConstants.SHOOTING_WHILE_MOVING_ENABLED || shooter == null) {
            return getDegreesToGoal();
        }
        double[] future = shooter.getShootingWhileMovingFuturePose();
        // getDegreesToGoalFromPosition subtracts futureH to get the servo angle,
        // but the servo is on the robot at the CURRENT heading. Correct by adding
        // back (futureH - curH). When stationary this is zero.
        double angle = getDegreesToGoalFromPosition(future[0], future[1], future[2]);
        angle += Math.toDegrees(future[2] - robot.cachedHeading);
        return angle;
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees > 180) degrees -= 360;
        else if (degrees < -180) degrees += 360;
        return degrees;
    }

    public void setTurretAngle(double targetAngleTurret) {
        targetAngleTurret = Math.max(
            TurretConstants.HARD_STOP_CCW,
            Math.min(TurretConstants.HARD_STOP_CW, targetAngleTurret)
        );

        rawTargetDegrees = targetAngleTurret;

        if (!smoothingInitialized) {
            smoothedTargetDegrees = targetAngleTurret;
            smoothingInitialized = true;
        } else {
            smoothedTargetDegrees = TurretConstants.SMOOTHING_ALPHA * targetAngleTurret
                                  + (1.0 - TurretConstants.SMOOTHING_ALPHA) * smoothedTargetDegrees;
        }

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

    public double getLastDegreesToGoal() {
        return lastDegreesToGoal;
    }

    @Override
    public void periodic() {
        if (ShooterConstants.MANUAL_OVERRIDE) {
            double manualAngle = CENTER + ((RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? getBlueTrackingOffset(robot.cachedPoseY)
                : getRedTrackingOffset(robot.cachedPoseY));
            setTurretDegree(manualAngle);
        } else {
            lastDegreesToGoal = getDegreesToGoalShootingWhileMoving();
            setTurretDegree(lastDegreesToGoal);
        }
        applyServoPosition(currentTargetDegrees);
    }
}
