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
    public static double runtimeVelocityOffset = 0;

    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    private FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();



    private ElapsedTime loopTimer = new ElapsedTime();
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

    private double shootingWhileMovingDistance = ShooterConstants.DEFAULT_DISTANCE;
    private double[] shootingWhileMovingFuturePose = new double[3]; // {x, y, headingRad}

    private double lastHoodServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;
    private double shotHoodCompensation = 0.0;
    private final double[] turretFieldPosTemp = new double[2];
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


    private double hoodAngleToServoPosition(double hoodAngleDegrees) {
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

    private double clampedPredict(double pos, double vel, double accel, double tof) {
        if (accel != 0.0 && vel != 0.0 && Math.signum(accel) != Math.signum(vel)) {
            double tStop = -vel / accel;
            if (tStop > 0 && tStop < tof) {
                return pos + vel * tStop + 0.5 * accel * tStop * tStop;
            }
        }
        return pos + vel * tof + 0.5 * accel * tof * tof;
    }

    private void updateShootingWhileMovingCompensation() {
        double curX = robot.cachedPoseX;
        double curY = robot.cachedPoseY;
        double curH = robot.cachedHeading;

        double dt = lastLoopDt;

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

        double fieldVelX = (curX - prevPoseX) / dt;
        double fieldVelY = (curY - prevPoseY) / dt;
        double velH = (curH - prevHeading) / dt;

        prevPoseX = curX;
        prevPoseY = curY;
        prevHeading = curH;

        double rawFieldVelX = fieldVelX;
        double rawFieldVelY = fieldVelY;
        double rawVelH = velH;

        double speed = Math.sqrt(fieldVelX * fieldVelX + fieldVelY * fieldVelY);
        if (speed < ShooterConstants.LEAD_VELOCITY_DEADBAND) {
            fieldVelX = 0.0;
            fieldVelY = 0.0;
        }
        if (Math.abs(velH) < ShooterConstants.LEAD_HEADING_VELOCITY_DEADBAND) {
            velH = 0.0;
        }
        {
            double dvx = rawFieldVelX - prevFieldVelX;
            double dvy = rawFieldVelY - prevFieldVelY;
            double dvMag = Math.sqrt(dvx * dvx + dvy * dvy);

            if (dvMag > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_VELOCITY_JUMP) {
                filteredAccelX = 0.0;
                filteredAccelY = 0.0;
                filteredAngularAccel = 0.0;
            } else {
                double rawAccelX = dvx / dt;
                double rawAccelY = dvy / dt;
                double rawAngularAccel = (rawVelH - prevHeadingVel) / dt;

                double alpha = ShooterConstants.SHOOTING_WHILE_MOVING_ACCEL_FILTER_ALPHA;
                filteredAccelX = alpha * rawAccelX + (1.0 - alpha) * filteredAccelX;
                filteredAccelY = alpha * rawAccelY + (1.0 - alpha) * filteredAccelY;
                filteredAngularAccel = alpha * rawAngularAccel + (1.0 - alpha) * filteredAngularAccel;

                double accelMag = Math.sqrt(filteredAccelX * filteredAccelX + filteredAccelY * filteredAccelY);
                if (accelMag < ShooterConstants.SHOOTING_WHILE_MOVING_ACCEL_DEADBAND) {
                    filteredAccelX = 0.0;
                    filteredAccelY = 0.0;
                }
                if (Math.abs(filteredAngularAccel) < ShooterConstants.SHOOTING_WHILE_MOVING_ANGULAR_ACCEL_DEADBAND) {
                    filteredAngularAccel = 0.0;
                }
            }

            double accelMag = Math.sqrt(filteredAccelX * filteredAccelX + filteredAccelY * filteredAccelY);
            if (accelMag > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ACCEL) {
                double scale = ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ACCEL / accelMag;
                filteredAccelX *= scale;
                filteredAccelY *= scale;
            }
            if (Math.abs(filteredAngularAccel) > ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL) {
                filteredAngularAccel = Math.signum(filteredAngularAccel) * ShooterConstants.SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL;
            }

            prevFieldVelX = rawFieldVelX;
            prevFieldVelY = rawFieldVelY;
            prevHeadingVel = rawVelH;
        }

        double tof = ShooterConstants.TIME_IN_AIR;

        shootingWhileMovingFuturePose[0] = clampedPredict(curX, fieldVelX, filteredAccelX, tof);
        shootingWhileMovingFuturePose[1] = clampedPredict(curY, fieldVelY, filteredAccelY, tof);
        shootingWhileMovingFuturePose[2] = clampedPredict(curH, velH, filteredAngularAccel, tof);

        double[] futureTurretPos = getTurretFieldPositionFrom(
                shootingWhileMovingFuturePose[0], shootingWhileMovingFuturePose[1], shootingWhileMovingFuturePose[2]);
        Pose goalPosition = FieldMap.getGoalPosition();
        double dx = goalPosition.getX() - futureTurretPos[0];
        double dy = goalPosition.getY() - futureTurretPos[1];
        shootingWhileMovingDistance = Math.sqrt(dx * dx + dy * dy);
        currentTimeInAir = tof;
    }

    public double[] getShootingWhileMovingFuturePose() {
        return shootingWhileMovingFuturePose;
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


    public void extendFlipper() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_EXTENDED);

        double step = (robot.cachedPoseY <= ShooterConstants.ShooterTuning.SHOT_HOOD_COMPENSATION_Y_THRESHOLD)
                ? ShooterConstants.ShooterTuning.SHOT_HOOD_COMPENSATION_STEP_BACK
                : ShooterConstants.ShooterTuning.SHOT_HOOD_COMPENSATION_STEP_FRONT;
        shotHoodCompensation += step;
    }

    public void resetHoodCompensation() {
        shotHoodCompensation = 0.0;
    }

    public void retractFlipper() {
        robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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

        double currentVelocity = robot.shooterMotor2.getVelocity();
        cachedVelocity = currentVelocity;

        double velocityError = targetVelocity - currentVelocity;
        lastError = velocityError;

        double totalPower;

        if (velocityError > ShooterConstants.ShooterTuning.RECOVERY_THRESHOLD) {
            currentControlMode = FlywheelControlMode.RECOVERY;
            double effectiveTarget = targetVelocity + ShooterConstants.ShooterTuning.RECOVERY_VELOCITY_BOOST;

            if (currentVelocity < effectiveTarget) {
                totalPower = 1.0;
            } else {
                double ff = ShooterConstants.ShooterTuning.kS * Math.signum(targetVelocity)
                          + ShooterConstants.ShooterTuning.kV * targetVelocity;
                totalPower = ff;
            }
            lastFfOutput = totalPower;
        } else {
            currentControlMode = FlywheelControlMode.MAINTAIN;
            double ff = ShooterConstants.ShooterTuning.kS * Math.signum(targetVelocity)
                      + ShooterConstants.ShooterTuning.kV * targetVelocity;
            double pCorrection = ShooterConstants.ShooterTuning.VELOCITY_kP * velocityError;
            totalPower = ff + pCorrection;
            lastFfOutput = ff;
        }

        totalPower = clamp(totalPower, 0, 1.0);

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
