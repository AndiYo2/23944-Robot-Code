package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import utility.RobotConstants;
import utility.RobotHardware;

public class Shooter implements Subsystem {

    private final double FLIPPER_POSITION_EXTEND = 1;
    private final double FLIPPER_POSITION_RETRACT = 0;
    RobotHardware robot;
    private final double[] txValues = new double[10];
    private int txValuesIndex = 0;
    private int txValuesCount = 0;

    private boolean limelightDisabled = false;

    public Shooter() {
        this.robot = RobotHardware.getInstance();;
    }

    // ============================================================
    // ====================== SHOOTER ==================
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
           Thread.sleep(600);
       } catch (InterruptedException e) {
           Thread.currentThread().interrupt();
       }
       robot.shooterFlipper.setPosition(FLIPPER_POSITION_RETRACT);
   }

    // ============================================================
    // ====================== VELOCITY CALCULATION ================
    // ============================================================

    // Configuration constants
    private static final double LIMELIGHT_HEIGHT = 0.41; // Height of limelight from ground in meters (adjust this)
    private static final double LIMELIGHT_ANGLE = 10.0; // Angle of limelight from horizontal in degrees (adjust this)
    private static final double TARGET_HEIGHT = 0.75; // Height of AprilTag center from ground
    private static final double LAUNCH_HEIGHT = 0.38; // Height of ball launch point from ground in meters (adjust this)
    private static final double LAUNCH_ANGLE = 40.0; // Launch angle in degrees (adjust based on your hood)
    private static final double GRAVITY = 9.81; // m/s^2

    private double requiredVelocity = 0; // Store calculated velocity

    /**
     * Calculate required shooter velocity based on distance to target
     * Uses Limelight ty (vertical angle) to calculate distance
     */
    public void calculateRequiredVelocity() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy(); // Vertical angle to target in degrees

            // Calculate horizontal distance to target using trigonometry
            double angleToTarget = LIMELIGHT_ANGLE + ty;
            double heightDifference = TARGET_HEIGHT - LIMELIGHT_HEIGHT;

            // Distance = height_difference / tan(angle)
            double horizontalDistance = heightDifference / Math.tan(Math.toRadians(angleToTarget));

            // Make sure distance is positive
            if (horizontalDistance <= 0) {
                horizontalDistance = 0.5; // Default minimum distance
            }

            // Calculate required velocity using projectile motion
            requiredVelocity = calculateVelocityForDistance(horizontalDistance);
        }
    }

    /**
     * Calculate required velocity to hit target at given horizontal distance
     * Uses projectile motion equations
     *
     * @param distance Horizontal distance to target in meters
     * @return Required velocity in radians/second for your motor
     */
    private double calculateVelocityForDistance(double distance) {
        double heightDiff = TARGET_HEIGHT - LAUNCH_HEIGHT;
        double angle = Math.toRadians(LAUNCH_ANGLE);

        // Projectile motion formula:
        // v = sqrt((g * d^2) / (2 * cos^2(θ) * (d * tan(θ) - h)))
        // where h is height difference, d is distance, θ is launch angle

        double numerator = GRAVITY * distance * distance;
        double denominator = 2 * Math.cos(angle) * Math.cos(angle) *
                (distance * Math.tan(angle) - heightDiff);

        if (denominator <= 0) {
            // Can't reach target with this angle
            return 100.0; // Return high default value
        }

        double velocityMS = Math.sqrt(numerator / denominator);

        // Convert to appropriate units for your motor
        // If your motor uses radians/second for the wheel, you'll need to convert
        // Assuming wheel diameter of 4 inches (0.1016 m):
        double wheelRadius = 0.0508; // 2 inches in meters
        double angularVelocity = velocityMS / wheelRadius;

        return angularVelocity;
    }

    /**
     * Get the currently calculated required velocity
     */
    public double getRequiredVelocity() {
        return requiredVelocity;
    }

    /**
     * Get horizontal distance to target in meters
     */
    public double getDistanceToTarget() {
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ty = result.getTy();
            double angleToTarget = LIMELIGHT_ANGLE + ty;
            double heightDifference = TARGET_HEIGHT - LIMELIGHT_HEIGHT;
            return heightDifference / Math.tan(Math.toRadians(angleToTarget));
        }

        return -1; // Invalid
    }
    // ============================================================
    // ====================== FULL METHODS ========================
    // ============================================================


    public boolean shootBall(){
        // Calculate required velocity based on current distance
        calculateRequiredVelocity();

        if(robot.shooterMotor.getVelocity(AngleUnit.RADIANS) > requiredVelocity * 0.95){ // 95% threshold
            if(robot.shooterFlipper.getPosition() == FLIPPER_POSITION_RETRACT){
                flip();
                return true;
            }
        }

        return false;
    }


    // ============================================================
    // ======================== TURRET SYSTEM ======================
    // ============================================================

    private static final double DEADBAND = 8.0;  // Stop moving when within 2 degrees
    private static final double POWER_SCALE = 0.015;  // How fast to turn (tune this)

    public void toggleLimelight(){
        limelightDisabled = !limelightDisabled;
    }


    @Override
    public void periodic() {

        // Calculate required velocity continuously
        calculateRequiredVelocity();

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