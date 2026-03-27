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
    RobotHardware robot;

    private Turret turret;

    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    private FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();



    private ElapsedTime loopTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastVelocityError = 0;
    private double lastVelocity = 0;

    // Default loop time when dt calculation fails (seconds)
    private static final double DEFAULT_LOOP_TIME = 0.02;
    private double cachedVelocity = 0;
    private double lastFfOutput = 0;
    private double lastPidOutput = 0;
    private double lastTotalPower = 0;
    private double lastError = 0;
    private double lastAcceleration = 0;

    private InterpLUT velocityLUT;
    private InterpLUT hoodLUT;

    private double requiredHoodAngle = ShooterConstants.HOOD_DEFAULT_ANGLE;

    // Lead-compensated state (shared with Turret via getter)
    private double leadAdjustedDistance = ShooterConstants.DEFAULT_DISTANCE;
    private double currentTimeInAir = 0.0;
    private double[] futurePose = new double[3]; // {x, y, headingRad}

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
        return velocityLUT.get(distance);
    }

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

    private double[] getTurretFieldPositionFrom(double robotX, double robotY, double robotHeadingRad) {
        double combinedAngle = robotHeadingRad + TURRET_OFFSET_ANGLE;
        turretFieldPosTemp[0] = robotX + TURRET_OFFSET_MAG * Math.sin(combinedAngle);
        turretFieldPosTemp[1] = robotY - TURRET_OFFSET_MAG * Math.cos(combinedAngle);
        return turretFieldPosTemp;
    }

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
        return Math.abs(requiredVelocity - cachedVelocity) < ShooterConstants.VELOCITY_TOLERANCE;
    }

    public double getVelocityError() {
        return lastError;
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

        if (ShooterConstants.ShooterTuning.TUNING_MODE) {
            requiredVelocity = ShooterConstants.ShooterTuning.TUNING_VELOCITY;
            requiredHoodAngle = ShooterConstants.ShooterTuning.TUNING_HOOD_ANGLE;
        } else {
            updateVelocityFromDistance();
        }
        setHoodAngle(requiredHoodAngle);
        double targetVelocity = requiredVelocity;

        //  read motor 2 only until rebuild
        double currentVelocity = robot.shooterMotor2.getVelocity();
        cachedVelocity = currentVelocity;

        double velocityError = targetVelocity - currentVelocity;
        lastError = velocityError;

        double measuredAcceleration = (currentVelocity - lastVelocity) / dt;
        lastVelocity = currentVelocity;
        lastAcceleration = measuredAcceleration;

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
        double pOutput = ShooterConstants.VELOCITY_kP * velocityError;

        integralSum += velocityError * dt;
        integralSum = clamp(integralSum, -ShooterConstants.INTEGRAL_MAX, ShooterConstants.INTEGRAL_MAX);

        // Zero-crossing reset prevents overshoot
        if (lastVelocityError != 0 && Math.signum(velocityError) != Math.signum(lastVelocityError)) {
            integralSum = 0;
        }
        double iOutput = ShooterConstants.VELOCITY_kI * integralSum;

        double dOutput = ShooterConstants.VELOCITY_kD * -measuredAcceleration;

        lastVelocityError = velocityError;

        double pidOutput = pOutput + iOutput + dOutput;
        lastPidOutput = pidOutput;

        // ==================== TOTAL OUTPUT ====================
        double totalPower = ffOutput + pidOutput;
        totalPower = clamp(totalPower, 0, 1.0);

        // Voltage compensation: scale power up as battery sags below nominal
        if (ShooterConstants.VOLTAGE_COMPENSATION_ENABLED && robot.voltageSensor != null) {
            double batteryVoltage = robot.voltageSensor.getVoltage();
            if (batteryVoltage > 0) {
                totalPower *= ShooterConstants.NOMINAL_VOLTAGE / batteryVoltage;
                totalPower = clamp(totalPower, 0, 1.0);
            }
        }

        lastTotalPower = totalPower;

        robot.shooterMotor1.setPower(totalPower);
        robot.shooterMotor2.setPower(totalPower);

        flipperStateMachinePeriodic();
    }
}
