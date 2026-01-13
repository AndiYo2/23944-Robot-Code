package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
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
import Constants.ShooterFeedforwardConstants;

/**
 * Shooter subsystem using FTCLib Feedforward + PID control.
 *
 * This is a drop-in replacement for the regular Shooter class that uses
 * external feedforward + PID control instead of the built-in PIDF.
 *
 * BENEFITS:
 * - Faster control loop (runs at your loop rate, not REV Hub's ~15Hz)
 * - Better disturbance rejection (faster recovery after shooting)
 * - More tuning flexibility with separate feedforward and feedback
 * - Proactive control (feedforward) + reactive correction (PID)
 *
 * To use: Replace "new Shooter()" with "new ShooterFeedforward()" in your OpMode
 */
public class ShooterFeedforward implements Subsystem {
    // Hardware reference
    private RobotHardware robot;

    // Turret and Odometry references
    private Turret turret;
    private Odometry odometry;

    // FTCLib controllers
    private SimpleMotorFeedforward feedforward;
    private PIDController pidController;

    // Flywheel velocity (ticks/sec)
    private double requiredVelocity = ShooterConstants.DEFAULT_VELOCITY;

    // Ball flipper state machine
    private FlickState currentState = FlickState.Idle;
    private ElapsedTime flickerTimer = new ElapsedTime();

    // Manual velocity override
    private boolean manualVelocityMode = false;
    private double manualVelocity = ShooterConstants.DEFAULT_VELOCITY;

    // Loop timing
    private ElapsedTime loopTimer = new ElapsedTime();

    // Telemetry data (for debugging)
    private double lastFfOutput = 0;
    private double lastPidOutput = 0;
    private double lastPower = 0;
    private double lastError = 0;

    public ShooterFeedforward() {
        this.robot = RobotHardware.getInstance();

        // Configure motors for EXTERNAL control (RUN_WITHOUT_ENCODER)
        robot.shooterMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.shooterMotor2.setDirection(DcMotor.Direction.REVERSE);
        robot.shooterMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Initialize FTCLib controllers
        initializeControllers();

        // Start with ball flipper retracted
        robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);

        loopTimer.reset();
    }

    private void initializeControllers() {
        feedforward = new SimpleMotorFeedforward(
            ShooterFeedforwardConstants.kS,
            ShooterFeedforwardConstants.kV,
            ShooterFeedforwardConstants.kA
        );

        pidController = new PIDController(
            ShooterFeedforwardConstants.kP,
            ShooterFeedforwardConstants.kI,
            ShooterFeedforwardConstants.kD
        );
    }

    public void setTurret(Turret turret) {
        this.turret = turret;
    }

    public void setOdometry(Odometry odometry) {
        this.odometry = odometry;
    }

    public double getDistanceToTarget() {
        if (turret == null) return 0;
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
                if (flickerTimer.seconds() < ShooterConstants.FLICK_TIME) break;
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
     * Check if flywheel is at target velocity (within tolerance)
     */
    public boolean isAtTargetVelocity() {
        double currentVelocity = robot.shooterMotor1.getVelocity();
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;
        return Math.abs(targetVelocity - currentVelocity) < ShooterFeedforwardConstants.VELOCITY_TOLERANCE;
    }

    /**
     * Get current velocity error
     */
    public double getVelocityError() {
        return lastError;
    }

    /**
     * Get telemetry data for debugging
     */
    public double[] getControllerOutputs() {
        return new double[] { lastFfOutput, lastPidOutput, lastPower };
    }

    public void setManualVelocity(double velocity) {
        manualVelocityMode = true;
        manualVelocity = velocity;
    }

    public void disableManualVelocity() {
        manualVelocityMode = false;
    }

    @Override
    public void periodic() {
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Update controllers with latest constants (allows live tuning)
        feedforward = new SimpleMotorFeedforward(
            ShooterFeedforwardConstants.kS,
            ShooterFeedforwardConstants.kV,
            ShooterFeedforwardConstants.kA
        );

        pidController.setPID(
            ShooterFeedforwardConstants.kP,
            ShooterFeedforwardConstants.kI,
            ShooterFeedforwardConstants.kD
        );

        // Get target velocity
        double targetVelocity = manualVelocityMode ? manualVelocity : requiredVelocity;
        if (!manualVelocityMode) {
            updateVelocityFromDistance();
            targetVelocity = requiredVelocity;
        }

        // Get current velocity (average of both motors)
        double currentVelocity = (robot.shooterMotor1.getVelocity() + robot.shooterMotor2.getVelocity()) / 2.0;

        // Calculate feedforward (proactive control)
        lastFfOutput = feedforward.calculate(targetVelocity);

        // Calculate PID (reactive correction)
        lastPidOutput = pidController.calculate(currentVelocity, targetVelocity);

        // Combine outputs
        lastPower = lastFfOutput + lastPidOutput;

        // Clamp to valid range
        lastPower = Math.max(0, Math.min(1.0, lastPower));

        // Track error
        lastError = targetVelocity - currentVelocity;

        // Apply power to motors
        robot.shooterMotor1.setPower(lastPower);
        robot.shooterMotor2.setPower(lastPower);

        // Run flipper state machine
        flipperStateMachinePeriodic();

        // Update turret with current field state from odometry
        if (turret != null && odometry != null) {
            turret.turretPeriodic(odometry.getFieldState());
        }
    }
}