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

public class Turret extends SubsystemBase {
    RobotHardware robot;

    private double currentTargetDegrees = 0.0;

    private double smoothedTargetDegrees = 0.0;
    private boolean smoothingInitialized = false;

    private Shooter shooter;

    private double lastDegreesToGoal = 0.0;

    private double lastServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

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

        double robotHeadingRad = robot.cachedHeading;
        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        turretAngleDeg += (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? getBlueTrackingOffset(robot.cachedPoseY)
                : getRedTrackingOffset(robot.cachedPoseY);

        return turretAngleDeg;
    }

    public double getDegreesToGoalFromPosition(double robotX, double robotY, double robotHeadingRad) {
        Pose goalPosition = FieldMap.getGoalPosition();

        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        double turretX = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        double turretY = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);

        double deltaX = goalPosition.getX() - turretX;
        double deltaY = goalPosition.getY() - turretY;

        double fieldAngleRad = Math.atan2(deltaY, deltaX);

        double turretAngleRad = fieldAngleRad - robotHeadingRad;

        double turretAngleDeg = Math.toDegrees(turretAngleRad);
        turretAngleDeg = normalizeAngle(turretAngleDeg);

        turretAngleDeg += (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                ? getBlueTrackingOffset(robotY)
                : getRedTrackingOffset(robotY);

        return turretAngleDeg;
    }

    public double getDegreesToGoalShootingWhileMoving() {
        if (!ShooterConstants.SHOOTING_WHILE_MOVING_ENABLED || shooter == null) {
            return getDegreesToGoal();
        }
        double[] future = shooter.getShootingWhileMovingFuturePose();
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
