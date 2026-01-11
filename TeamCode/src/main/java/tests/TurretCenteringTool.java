package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.TurretConstants;

/**
 * Turret Centering Tool - Simplified version using Panels dashboard.
 *
 * This OpMode holds the turret at exactly 0° (center position).
 * Useful for verifying encoder calibration or testing turret control.
 *
 * Adjust PID values in Panels dashboard:
 * - TurretConstants.TURRET_PID (PIDCoefficients object with p, i, d)
 *
 * CONTROLS:
 *   A: Toggle Motor ON/OFF
 *   X: Emergency Stop
 */
@TeleOp(name = "Turret Centering Tool", group = "Tests")
public class TurretCenteringTool extends OpMode {

    private RobotHardware robot;

    // PID variables
    private double targetPosition = 0.0;  // Always 0° (center)

    private double lastError = 0;
    private double integral = 0;
    private long lastTime = 0;

    private boolean motorEnabled = true;  // Toggle motor on/off
    private boolean aButtonPressed = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        lastTime = System.nanoTime();

        telemetry.addLine("========================================");
        telemetry.addLine("   TURRET VERIFICATION TOOL");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addLine("Holds turret at 0° (center)");
        telemetry.addLine("Adjust PID in Panels dashboard");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  A: Toggle Motor ON/OFF");
        telemetry.addLine("  X: Emergency Stop");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTime = System.nanoTime();
        lastError = 0;
        integral = 0;
    }

    @Override
    public void loop() {
        // Refresh values from Panels dashboard
        PanelsConfigurables.INSTANCE.refreshClass(RobotConstants.class);

        // EMERGENCY STOP - X button
        if (gamepad1.x) {
            robot.turretServo.setPower(0);
            telemetry.addLine("========================================");
            telemetry.addLine("        EMERGENCY STOP");
            telemetry.addLine("========================================");
            telemetry.addLine("Motor power set to 0");
            telemetry.addLine("Press STOP to exit");
            telemetry.update();
            return;
        }

        // TOGGLE MOTOR - A button
        if (gamepad1.a && !aButtonPressed) {
            motorEnabled = !motorEnabled;
            if (!motorEnabled) {
                robot.turretServo.setPower(0);  // Immediately stop motor when disabled
                integral = 0;  // Reset integral when toggling off
                lastError = 0;
            }
        }
        aButtonPressed = gamepad1.a;

        // Get current position (using current offset system)
        double currentPosition = getCurrentPositionWithOffset();

        // Calculate error (target is always 0°)
        double error = targetPosition - currentPosition;

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        // Sanity check on dt
        if (dt > RobotConstants.PID.DT_MAX || dt < RobotConstants.PID.DT_MIN) {
            dt = RobotConstants.PID.DT_DEFAULT;
        }

        // Get PID coefficients from RobotConstants
        double kP = TurretConstants.TURRET_PID.p;
        double kI = TurretConstants.TURRET_PID.i;
        double kD = TurretConstants.TURRET_PID.d;

        // PID calculations (always calculate for telemetry, but only apply if motor enabled)
        double power = 0;

        if (motorEnabled) {
            integral += error * dt;
            integral = Math.max(RobotConstants.PID.INTEGRAL_CLAMP_MIN, Math.min(RobotConstants.PID.INTEGRAL_CLAMP_MAX, integral));
            double derivative = (error - lastError) / dt;
            lastError = error;

            power = (kP * error) + (kI * integral) + (kD * derivative);

            // Clamp power
            power = Math.max(-TurretConstants.TURRET_POWER_LIMIT_NORMAL, Math.min(TurretConstants.TURRET_POWER_LIMIT_NORMAL, power));

            // Set motor power
            robot.turretServo.setPower(power);
        } else {
            // Motor disabled - ensure power is 0
            robot.turretServo.setPower(0);
        }

        // Get raw encoder reading for centering
        double rawEncoderReading = getRawEncoderReading();

        // Display telemetry
        telemetry.addLine("========================================");
        if (motorEnabled) {
            telemetry.addLine("   MOTOR ON - HOLDING AT CENTER");
        } else {
            telemetry.addLine("   MOTOR OFF - MANUAL POSITIONING");
        }
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addData("Motor Status", motorEnabled ? "ON (holding position)" : "OFF (free to move)");
        telemetry.addLine();
        telemetry.addData("Target Position", "0.00° (center)");
        telemetry.addData("Current Position", "%.2f°", currentPosition);
        telemetry.addData("Position Error", "%.2f°", error);
        telemetry.addLine();
        telemetry.addLine("--- PID (adjust in Panels) ---");
        telemetry.addData("P", "%.5f", kP);
        telemetry.addData("I", "%.5f", kI);
        telemetry.addData("D", "%.5f", kD);
        telemetry.addLine();
        telemetry.addLine("========================================");
        telemetry.addLine("    ENCODER VERIFICATION:");
        telemetry.addLine("========================================");
        telemetry.addData("Raw Encoder Reading", "%.2f°", rawEncoderReading);
        telemetry.addData("Expected at Center", "0° (or 360°)");
        telemetry.addLine();

        // Show calibration status
        if (Math.abs(rawEncoderReading) < 5.0 || Math.abs(rawEncoderReading - RobotConstants.Encoder.FULL_ROTATION_DEGREES) < 5.0) {
            telemetry.addLine("ENCODER CENTERED CORRECTLY!");
        } else if (rawEncoderReading > RobotConstants.Encoder.ANGLE_UPPER_BOUND) {
            telemetry.addLine(String.format("Encoder off by %.1f° - needs recalibration", rawEncoderReading));
        } else {
            telemetry.addLine(String.format("Encoder off by %.1f° - needs recalibration", Math.abs(rawEncoderReading)));
        }
        telemetry.addLine();

        telemetry.addLine("========================================");
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A Button", "Toggle Motor: %s", motorEnabled ? "ON->OFF" : "OFF->ON");
        telemetry.addLine("X Button: Emergency Stop");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    /**
     * Gets raw encoder reading in degrees (0-360 range, no offset applied)
     */
    private double getRawEncoderReading() {
        double voltage = robot.turretEncoder.getVoltage();
        double degrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Normalize to [0, 360] for easier reading
        while (degrees < 0) degrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (degrees >= RobotConstants.Encoder.FULL_ROTATION_DEGREES) degrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        return degrees;
    }

    /**
     * Gets current encoder position (no offset needed - encoder is centered!)
     * This matches the Shooter.java approach
     */
    private double getCurrentPositionWithOffset() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Normalize to [-180, 180]
        while (rawDegrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) rawDegrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (rawDegrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) rawDegrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Encoder is centered - no offset needed!
        return rawDegrees;
    }

    @Override
    public void stop() {
        robot.turretServo.setPower(0);
    }
}
