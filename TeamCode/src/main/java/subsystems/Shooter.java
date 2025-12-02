package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import utility.RobotHardware;

public class Shooter implements Subsystem {

    RobotHardware robot;

    public Shooter() {
        this.robot = RobotHardware.getInstance();
    }

    // ============================================================
    // ====================== SHOOTER (unchanged) ==================
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

    // ****** STAGING MOTOR ******

    public void setStagingMotorSpeed(double speed) {
        robot.shooterBeltMotor.setVelocity(speed, AngleUnit.RADIANS);
    }

    public double getStagingMotorSpeed() {
        return robot.shooterBeltMotor.getVelocity(AngleUnit.RADIANS);
    }

    public void setStagingMotorPower(double power) {
        robot.shooterBeltMotor.setPower(power);
    }

    public double getStagingMotorPower() {
        return robot.shooterBeltMotor.getPower();
    }

    public void stopStagingMotor() {
        robot.shooterBeltMotor.setPower(0);
    }

    // ============================================================
    // ======================== TURRET SYSTEM ======================
    // ============================================================

    // MUCH more aggressive gains for faster response
    private double turretKp = 0.035;      // Increased from 0.01
    private double turretKi = 0.0001;     // Small I term to eliminate steady-state error
    private double turretKd = 0.003;      // Increased from 0.0005 for damping

    private double turretIntegral = 0;
    private double turretPrevError = 0;
    private double turretTarget = 0;

    // Tighter deadband and better filtering
    private static final double ERROR_DEADBAND = .8;     // Tighter tolerance
    private static final double INTEGRAL_MAX = 30.0;
    private static final double LIMELIGHT_DEADBAND = .3; // More responsive to limelight
    private static final double OUTPUT_DEADZONE = 0.03;   // Very small deadzone

    // Low-pass filter for Limelight data
    private double filteredTx = 0;
    private static final double FILTER_ALPHA = 0.3;  // 0 = heavy filter, 1 = no filter

    private long lastUpdateTime = 0;
    private int stableFrames = 0;  // Count frames we've been stable

    /** Current turret angle (0–360 deg) */
    public double getTurretAngle() {
        return (robot.shooterEncoder.getVoltage() / 3.3) * 360.0;
    }

    /** Set new turret setpoint */
    private void setTurretTargetInternal(double angleDeg) {
        turretTarget = wrap(angleDeg);
    }

    /** Wrap angle to 0-360 */
    private double wrap(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    /** Signed angle error (-180 to 180) */
    private double angleError(double target, double current) {
        double error = wrap(target) - wrap(current);
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        return error;
    }

    // ============================================================
    // ================== ALWAYS AUTO-AIM PERIODIC =================
    // ============================================================

    @Override
    public void periodic() {
        long currentTime = System.currentTimeMillis();
        double dt = (lastUpdateTime == 0) ? 0.02 : (currentTime - lastUpdateTime) / 1000.0;
        lastUpdateTime = currentTime;

        // ===================== 1. AUTO-AIM WITH FILTERING ======================
        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = result.getTx();

            // Apply low-pass filter to smooth out noisy Limelight data
            filteredTx = (FILTER_ALPHA * tx) + ((1 - FILTER_ALPHA) * filteredTx);

            // Only update if filtered error is significant
            if (Math.abs(filteredTx) > LIMELIGHT_DEADBAND) {
                double current = getTurretAngle();
                setTurretTargetInternal(current + filteredTx);
                stableFrames = 0;  // Reset stability counter
            }
        }

        // ===================== 2. PID CONTROL ====================
        double currentAngle = getTurretAngle();
        double error = angleError(turretTarget, currentAngle);

        // Check if we're stable
        if (Math.abs(error) < ERROR_DEADBAND) {
            stableFrames++;

            // Only stop if we've been stable for a few frames (prevents flickering)
            if (stableFrames > 3) {
                turretIntegral = 0;
                robot.turretServo.setPower(0);
                turretPrevError = 0;
                return;
            }
        } else {
            stableFrames = 0;
        }

        // Integral with anti-windup (only accumulate if not saturated)
        turretIntegral += error * dt;
        turretIntegral = Range.clip(turretIntegral, -INTEGRAL_MAX, INTEGRAL_MAX);

        // Derivative with filtering
        double derivative = (dt > 0) ? (error - turretPrevError) / dt : 0;

        // PID output
        double output = (turretKp * error)
                + (turretKi * turretIntegral)
                + (turretKd * derivative);

        // Small deadzone to prevent tiny oscillations
        if (Math.abs(output) < OUTPUT_DEADZONE) {
            output = 0;
        }

        output = Range.clip(output, -1, 1);
        robot.turretServo.setPower(output);

        turretPrevError = error;
    }
}