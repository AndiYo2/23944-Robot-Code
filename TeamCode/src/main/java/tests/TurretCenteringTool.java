package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotConstants;
import utility.RobotHardware;

/**
 * Turret Centering Tool
 *
 * This OpMode holds the turret at exactly 0° (center position).
 * Useful for verifying encoder calibration or testing turret control.
 *
 * The encoder has been centered - this tool can now be used to:
 * - Verify the turret holds position correctly
 * - Test PID tuning
 * - Confirm encoder reads 0° at center
 *
 * Press A to toggle motor ON/OFF
 * Press X for emergency stop
 */
@TeleOp(name = "Turret Centering Tool", group = "Tests")
public class TurretCenteringTool extends OpMode {

    private RobotHardware robot;

    // PID variables
    private double targetPosition = 0.0;  // Always 0° (center)
    private double kP = RobotConstants.Shooter.TURRET_PID.p;
    private double kI = RobotConstants.Shooter.TURRET_PID.i;
    private double kD = RobotConstants.Shooter.TURRET_PID.d;

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
        telemetry.addLine("Encoder is centered! ✓");
        telemetry.addLine();
        telemetry.addLine("This tool holds turret at 0° (center)");
        telemetry.addLine("Use it to:");
        telemetry.addLine("  • Verify encoder reads 0° at center");
        telemetry.addLine("  • Test PID holding performance");
        telemetry.addLine("  • Confirm mechanical alignment");
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
        // EMERGENCY STOP - X button
        if (gamepad1.x) {
            robot.turretServo.setPower(0);
            telemetry.addLine("========================================");
            telemetry.addLine("        ⚠ EMERGENCY STOP ⚠");
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
        if (dt > 1.0 || dt < 0.001) {
            dt = 0.02; // Default to 50Hz
        }

        // PID calculations (always calculate for telemetry, but only apply if motor enabled)
        double power = 0;

        if (motorEnabled) {
            integral += error * dt;
            integral = Math.max(-50, Math.min(50, integral)); // Anti-windup
            double derivative = (error - lastError) / dt;
            lastError = error;

            power = (kP * error) + (kI * integral) + (kD * derivative);

            // Clamp power
            power = Math.max(-0.5, Math.min(0.5, power));

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
        telemetry.addLine("========================================");
        telemetry.addLine("    ENCODER VERIFICATION:");
        telemetry.addLine("========================================");
        telemetry.addData("Raw Encoder Reading", "%.2f°", rawEncoderReading);
        telemetry.addData("Expected at Center", "0° (or 360°)");
        telemetry.addLine();

        // Show calibration status
        if (Math.abs(rawEncoderReading) < 5.0 || Math.abs(rawEncoderReading - 360.0) < 5.0) {
            telemetry.addLine("✓ ENCODER CENTERED CORRECTLY!");
        } else if (rawEncoderReading > 180) {
            telemetry.addLine(String.format("⚠ Encoder off by %.1f° - needs recalibration", rawEncoderReading));
        } else {
            telemetry.addLine(String.format("⚠ Encoder off by %.1f° - needs recalibration", Math.abs(rawEncoderReading)));
        }
        telemetry.addLine();

        telemetry.addLine("========================================");
        telemetry.addData("Motor Power", "%.3f", power);
        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A Button", "Toggle Motor: %s", motorEnabled ? "ON→OFF" : "OFF→ON");
        telemetry.addLine("X Button: Emergency Stop");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    /**
     * Gets raw encoder reading in degrees (0-360 range, no offset applied)
     */
    private double getRawEncoderReading() {
        double voltage = robot.turretEncoder.getVoltage();
        double degrees = (voltage / 3.3) * 360.0;

        // Normalize to [0, 360] for easier reading
        while (degrees < 0) degrees += 360;
        while (degrees >= 360) degrees -= 360;

        return degrees;
    }

    /**
     * Gets current encoder position (no offset needed - encoder is centered!)
     * This matches the Shooter.java approach
     */
    private double getCurrentPositionWithOffset() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Normalize to [-180, 180]
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        // Encoder is centered - no offset needed!
        return rawDegrees;
    }

    @Override
    public void stop() {
        robot.turretServo.setPower(0);
    }
}
