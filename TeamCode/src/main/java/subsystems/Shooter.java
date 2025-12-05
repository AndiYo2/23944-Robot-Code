package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import utility.RobotConstants;
import utility.RobotHardware;

public class Shooter implements Subsystem {

    private final double FLIPPER_POSITION_EXTEND = .6;
    private final double FLIPPER_POSITION_RETRACT = .325;
    RobotHardware robot;
    private final double[] txValues = new double[10];
    private int txValuesIndex = 0;
    private int txValuesCount = 0;

    private boolean limelightDisabled = false;

    public Shooter() {
        this.robot = RobotHardware.getInstance();;
    }

    // ============================================================
    // ====================== SHOOTER =============================
    // ============================================================

    public double getShooterMotorVelocity() {
        return robot.shooterMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setShooterMotorVelocity(double speed) {
        robot.shooterMotor.setVelocity(speed, AngleUnit.RADIANS);
    }

    public double getShooterPower() {
        return robot.shooterMotor.getPower();
    }

    public void setShooterPower(double power) {
        robot.shooterMotor.setPower(power);
    }

    public void stopShooterMotor() {
        robot.shooterMotor.setPower(0);
    }

    // ============================================================
    // ====================== STAGING FLIPPER =====================
    // ============================================================

    public void flip(){
        robot.shooterFlipper.setPosition(FLIPPER_POSITION_EXTEND);
        try {
            Thread.sleep(400);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        robot.shooterFlipper.setPosition(FLIPPER_POSITION_RETRACT);
    }

    // ============================================================
    // ====================== VELOCITY CALCULATION ================
    // ============================================================

    // Configuration constants
    private static final double LIMELIGHT_HEIGHT = 0.41; // Height of limelight from ground in meters
    private static final double LIMELIGHT_ANGLE = 10.0; // Angle of limelight from horizontal in degrees
    private static final double APRILTAG_HEIGHT = 0.75; // Height of AprilTag center from ground (could be 1.0m - verify this!)
    private static final double TARGET_OFFSET = .1; // How much higher than AprilTag we want to aim (meters)
    private static final double TARGET_HEIGHT = APRILTAG_HEIGHT + TARGET_OFFSET; // Actual target height (1.0m or 1.25m)
    private static final double LAUNCH_HEIGHT = 0.38; // Height of ball launch point from ground in meters
    private static final double LAUNCH_ANGLE = 40.0; // Launch angle in degrees (adjust based on your hood)
    private static final double GRAVITY = 9.81; // m/s^2

    // Velocity lookup table - every 0.05m from 0.5m to 1.5m
    // Format: {distance in meters, velocity in rad/s}
    // Tune each value by testing at that exact distance
    private static final double[][] VELOCITY_MAP = {
            {0.50, 23.0},
            {0.55, 24.0},
            {0.60, 25.0},
            {0.65, 26.0},
            {0.70, 27.0},
            {0.75, 29.0},
            {0.80, 31.0},
            {0.85, 33.5},
            {0.90, 36.0},
            {0.95, 38.0},
            {1.00, 40.0},
            {1.05, 42.0},
            {1.10, 44.0},
            {1.15, 46.0},
            {1.20, 48.0},
            {1.25, 50.0},
            {1.30, 51.0},
            {1.35, 53.0},
            {1.40, 54.0},
            {1.45, 54.5},
            {1.50, 55.0}
    };

    private double requiredVelocity = 25; // Store calculated velocity

    /**
     * Calculate required shooter velocity based on distance to target
     * Finds the closest entry in the lookup table (rounded to nearest 0.05m)
     */
    public void calculateRequiredVelocity() {
        double distance = getDistanceToTarget();

        if (distance > 0) {
            // Round distance to nearest 0.05m
            double roundedDistance = Math.round(distance / 0.05) * 0.05;

            // Find exact match in table
            double closestVelocity = VELOCITY_MAP[VELOCITY_MAP.length / 2][1]; // default to middle
            double closestDistanceDiff = Double.MAX_VALUE;

            for (int i = 0; i < VELOCITY_MAP.length; i++) {
                double distDiff = Math.abs(VELOCITY_MAP[i][0] - roundedDistance);
                if (distDiff < closestDistanceDiff) {
                    closestDistanceDiff = distDiff;
                    closestVelocity = VELOCITY_MAP[i][1];
                }
            }

            requiredVelocity = closestVelocity;

        } else {
            // No valid target - use middle value from lookup table
            requiredVelocity = VELOCITY_MAP[VELOCITY_MAP.length / 2][1];
        }
    }

    /**
     * Get the currently calculated required velocity
     */
    public double getRequiredVelocity() {
        return requiredVelocity;
    }

    public void lowerRequiredVelocity(){
        requiredVelocity -= .5;
    }

    public void raiseRequiredVelocity(){
        requiredVelocity += .5;
    }

    /**
     * Get horizontal distance to target in meters
     */
    public double getDistanceToTarget() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToTarget = LIMELIGHT_ANGLE + ty;
            double heightDifference = APRILTAG_HEIGHT - LIMELIGHT_HEIGHT;
            double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));
            return distance > 0 ? distance : -1;
        }

        return -1; // Invalid
    }

    /**
     * Check if shooter is ready to fire
     */

    // ============================================================
    // ====================== FULL METHODS ========================
    // ============================================================

    public boolean shootBall(){
        // Calculate required velocity based on current distance
        // Check if shooter is at speed and flipper is ready

        flip();
        return true;

    }


    // ============================================================
    // ======================== TURRET SYSTEM ======================
    // ============================================================

    private static final double DEADBAND = 8.0;  // Stop moving within 8 degrees
    private static final double POWER_SCALE = 0.015;  // How fast to turn (tune this)

    public void toggleLimelight(){
        limelightDisabled = !limelightDisabled;
    }

    public boolean limelightDisabled(){
        return limelightDisabled;
    }


    @Override
    public void periodic() {

        // Calculate required velocity continuously


        robot.shooterMotor.setVelocity(requiredVelocity, AngleUnit.RADIANS);


        // Get Limelight data
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid() && !limelightDisabled) {
            double tx = result.getTx();  // Raw horizontal error in degrees

            // Store tx value in circular buffer
            txValues[txValuesIndex] = tx;
            txValuesIndex = (txValuesIndex + 1) % 10;
            txValuesCount = Math.min(txValuesCount + 1, 10);

            // Wait until we have enough samples
            if (txValuesCount < 10) {
                robot.turretServo.setPower(0);
                return;
            }

            // Calculate average tx
            double avgTx = 0;
            for (double value : txValues) {
                avgTx += value;
            }
            avgTx /= 10;

            // If we're close enough, stop
            if (Math.abs(avgTx) < DEADBAND) {
                robot.turretServo.setPower(0);
                return;
            }

            // Calculate power proportional to error
            double power = avgTx * POWER_SCALE;

            // Limit max power to prevent wild movements
            if (power > 0.5) power = 0.5;
            if (power < -0.5) power = -0.5;

            // Apply power (positive tx = target is right, so turn right)
            robot.turretServo.setPower(power);
        } else {
            // No target visible, stop moving
            robot.turretServo.setPower(0);
        }
    }
}