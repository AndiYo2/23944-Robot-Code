package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.RobotConstants;
import utility.RobotHardware;
import Constants.ShooterConstants;

import static Constants.ShooterConstants.FLICK_TIME;

public class Shooter implements Subsystem {
    // Hardware reference
    RobotHardware robot;

    // Turret subsystem reference
    private Turret turret;

    // Odometry subsystem reference
    private Odometry odometry;

    // Flywheel velocity (ticks/sec) - updated each loop based on distance to goal
    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    // Ball flipper state machine
    FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();

    // Manual velocity override (for tuning/testing)
    private boolean manualVelocityMode = false;
    private double manualVelocity = ShooterConstants.DEFAULT_VELOCITY;

    // Custom feedforward + PID state
    private ElapsedTime loopTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastVelocityError = 0;
    private double lastVelocity = 0;

    // Default loop time when dt calculation fails (seconds)
    private static final double DEFAULT_LOOP_TIME = 0.02;

    // Telemetry data for tuning
    private double lastFfOutput = 0;
    private double lastPidOutput = 0;
    private double lastTotalPower = 0;
    private double lastError = 0;
    private double lastAcceleration = 0;


    public Shooter() {
        this.robot = RobotHardware.getInstance();

        // Configure motors for EXTERNAL control (RUN_WITHOUT_ENCODER)
        // We still read the encoder for velocity feedback, but we control power directly
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Flywheel motor 2 (reversed to spin same direction)
        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Start with ball flipper retracted
        robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);

        loopTimer.reset();
    }

    public void setTurret(Turret turret) {
        this.turret = turret;
    }

    public void setOdometry(Odometry odometry) {
        this.odometry = odometry;
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
        // Return fixed velocity during Limelight scan mode
        if (LimelightConstants.manuallySlowedForScan) {
            return ShooterConstants.FALLBACK_VELOCITY;
        }

        // Select correct lookup table based on alliance and zone
        boolean isBlue = (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue);
        double[][] table;

        if (isBlue) {
            table = (distance < ShooterConstants.BLUE_ZONE_BOUNDARY)
                ? ShooterConstants.BLUE_FRONT_LOOKUP
                : ShooterConstants.BLUE_BACK_LOOKUP;
        } else {
            table = (distance < ShooterConstants.RED_ZONE_BOUNDARY)
                ? ShooterConstants.RED_FRONT_LOOKUP
                : ShooterConstants.RED_BACK_LOOKUP;
        }

        return interpolateFromTable(table, distance);
    }

    private double interpolateFromTable(double[][] table, double distance) {
        // Clamp to table bounds
        if (distance <= table[0][0]) {
            return table[0][1];
        }
        int lastIndex = table.length - 1;
        if (distance >= table[lastIndex][0]) {
            return table[lastIndex][1];
        }

        // Linear interpolation between table entries
        for (int i = 0; i < table.length - 1; i++) {
            double dist1 = table[i][0];
            double dist2 = table[i + 1][0];

            if (distance >= dist1 && distance <= dist2) {
                double vel1 = table[i][1];
                double vel2 = table[i + 1][1];
                // Avoid division by zero if table entries have same distance
                if (dist2 == dist1) {
                    return vel1;
                }
                double ratio = (distance - dist1) / (dist2 - dist1);
                return vel1 + (vel2 - vel1) * ratio;
            }
        }

        return ShooterConstants.FALLBACK_VELOCITY;
    }

    private void updateVelocityFromDistance() {
        double distance = getDistanceToTarget();
        requiredVelocity = getVelocityFromDistance(distance);
    }

    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return;

        switch (currentState) {
            case Start:
                robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                currentState = FlickState.Idle;
                break;
        }
    }

    public void triggerShot() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
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
        double currentVelocity = robot.shooterMotor1.getVelocity();
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;
        return Math.abs(targetVelocity - currentVelocity) < ShooterConstants.VELOCITY_TOLERANCE;
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
        return (robot.shooterMotor1.getVelocity() + robot.shooterMotor2.getVelocity()) / 2.0;
    }

    /**
     * Get target velocity for telemetry.
     */
    public double getTargetVelocity() {
        return manualVelocityMode ? manualVelocity : requiredVelocity;
    }

    /**
     * Get controller outputs for tuning telemetry.
     * Returns [ffOutput, pidOutput, totalPower, acceleration]
     */
    public double[] getControllerOutputs() {
        return new double[] { lastFfOutput, lastPidOutput, lastTotalPower, lastAcceleration };
    }

    /**
     * Set manual velocity for tuning.
     */
    public void setManualVelocity(double velocity) {
        manualVelocityMode = true;
        manualVelocity = velocity;
    }

    /**
     * Disable manual velocity mode.
     */
    public void disableManualVelocity() {
        manualVelocityMode = false;
    }

    @Override
    public void periodic() {
        // Get loop time for derivative and integral calculations
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Prevent division by zero on first loop
        if (dt <= 0) dt = DEFAULT_LOOP_TIME;

        // Update target velocity from distance if not in manual mode
        if (!manualVelocityMode) {
            updateVelocityFromDistance();
        }
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;

        // Get current velocity (average of both motors for accuracy)
        double currentVelocity = (robot.shooterMotor1.getVelocity() + robot.shooterMotor2.getVelocity()) / 2.0;

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

        // Update turret with current field state from odometry
        if (turret != null && odometry != null) {
            turret.turretPeriodic(odometry.getFieldState());
        }
    }
}
