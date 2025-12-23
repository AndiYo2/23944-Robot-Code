package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import utility.RobotConstants;
import utility.RobotHardware;

@TeleOp(name = "TurretPIDF", group = "Tests")
public class TurretPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;

    // Target positions (in degrees, within safe limits)
    private double targetPosition = 0;
    private double[] targetPositions = {0, 45, 90, -45, -90, 180, -180};
    private int targetIndex = 0;

    // Manual angle adjustment (for fine control)
    private boolean manualMode = false;

    // PID coefficients - Starting values from spindexer
    private double P = 0.0122;
    private double I = 0;
    private double D = 0.0005;

    // PID calculation variables
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;
    private boolean firstLoop = true;

    // Step sizes for tuning
    private double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 2; // Start with 0.001

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        targetPosition = getCurrentPosition();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Max Safe Angle", "±%.0f°", RobotConstants.Shooter.MAX_TURRET_ANGLE);
        telemetry.addLine("========== CONTROLS ==========");
        telemetry.addLine("Y: Cycle Target Position");
        telemetry.addLine("A: Reset to Center (0°)");
        telemetry.addLine("X: EMERGENCY STOP");
        telemetry.addLine("Left Stick X: Manual Angle Adjust");
        telemetry.addLine("B: Change Step Size");
        telemetry.addLine("D-Pad Up/Down: Adjust P");
        telemetry.addLine("D-Pad Left/Right: Adjust D");
        telemetry.addLine("Left Bumper: Decrease I");
        telemetry.addLine("Right Bumper: Increase I");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTime = System.nanoTime() / 1e9;
        firstLoop = true;
    }

    @Override
    public void loop() {
        // EMERGENCY STOP - X button
        if (gamepad1.x) {
            robot.turretServo.setPower(0);
            telemetry.addLine("====== EMERGENCY STOP ======");
            telemetry.addLine("Press A to reset to center");
            telemetry.update();
            return;
        }

        // Handle input for changing target position
        if (gamepad1.y) {
            targetIndex = (targetIndex + 1) % targetPositions.length;
            targetPosition = targetPositions[targetIndex];
            manualMode = false;
            resetPID();
        }

        // Handle input for resetting to center
        if (gamepad1.a) {
            targetPosition = 0;
            manualMode = false;
            resetPID();
        }

        // Manual angle adjustment with left stick
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            manualMode = true;
            targetPosition += gamepad1.left_stick_x * 2.0; // 2 degrees per tick at full stick
        }

        // SAFETY: Clamp target position to safe limits
        targetPosition = Math.max(-RobotConstants.Shooter.MAX_TURRET_ANGLE,
                                  Math.min(RobotConstants.Shooter.MAX_TURRET_ANGLE, targetPosition));

        // Handle input for changing step size
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Handle P tuning
        if (gamepad1.dpad_up) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            P -= stepSizes[stepIndex];
            P = Math.max(0, P);
        }

        // Handle D tuning
        if (gamepad1.dpad_right) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_left) {
            D -= stepSizes[stepIndex];
            D = Math.max(0, D);
        }

        // Handle I tuning
        if (gamepad1.right_bumper) {
            I += stepSizes[stepIndex];
        }
        if (gamepad1.left_bumper) {
            I -= stepSizes[stepIndex];
            I = Math.max(0, I);
        }

        // Get current position
        double currentPosition = getCurrentPosition();

        // Calculate error (shortest path)
        double error = targetPosition - currentPosition;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Calculate dt
        double currentTime = System.nanoTime() / 1e9;
        double dt = currentTime - lastTime;

        // Skip first loop to avoid huge dt
        if (firstLoop) {
            firstLoop = false;
            lastTime = currentTime;
            lastError = error;
            return;
        }

        // Sanity check on dt
        if (dt > 1.0 || dt < 0.001) {
            dt = 0.02; // Default to 50Hz
        }

        lastTime = currentTime;

        // Update integral with anti-windup
        integral += error * dt;
        integral = Math.max(-50, Math.min(50, integral)); // Clamp integral

        // Calculate derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Calculate PID output
        double output = P * error + I * integral + D * derivative;

        // Clamp output
        output = Math.max(-1, Math.min(1, output));

        // Set motor power
        robot.turretServo.setPower(output);

        // Check if approaching limits
        boolean nearLimit = Math.abs(currentPosition) > RobotConstants.Shooter.TURRET_WARNING_ANGLE;
        boolean atLimit = Math.abs(targetPosition) >= RobotConstants.Shooter.MAX_TURRET_ANGLE;

        // Telemetry - Safety warnings first
        if (atLimit) {
            telemetry.addLine("⚠⚠⚠ AT MAXIMUM SAFE ANGLE ⚠⚠⚠");
        } else if (nearLimit) {
            telemetry.addLine("⚠ WARNING: Approaching limit!");
        }

        telemetry.addData("Target Position", "%.1f° %s", targetPosition,
                         manualMode ? "(MANUAL)" : "");
        telemetry.addData("Current Position", "%.1f°", currentPosition);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Safe Range", "±%.0f° (Max: ±%.0f°)",
                         RobotConstants.Shooter.TURRET_WARNING_ANGLE,
                         RobotConstants.Shooter.MAX_TURRET_ANGLE);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P", "%.5f (D-Pad U/D)", P);
        telemetry.addData("I", "%.5f (Bumpers)", I);
        telemetry.addData("D", "%.5f (D-Pad L/R)", D);
        telemetry.addData("Step Size", "%.5f (B)", stepSizes[stepIndex]);
        telemetry.addLine("-----------------------------");
        telemetry.addData("PID Output", "%.3f", output);
        telemetry.addData("Motor Power", "%.3f", robot.turretServo.getPower());
        telemetry.addData("Integral", "%.3f", integral);
        telemetry.addData("Derivative", "%.3f", derivative);
        telemetry.addData("dt", "%.4f", dt);
        telemetry.addData("Raw Voltage", "%.3fV", robot.turretEncoder.getVoltage());
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Press Y to cycle target position");
        telemetry.addLine("Press A to reset to center");
        telemetry.addLine("Current PID Coefficients:");
        telemetry.addData("", "new PIDCoefficients(%.5f, %.5f, %.5f)", P, I, D);
        telemetry.update();
    }

    /**
     * Gets the current turret position from the encoder
     * @return Turret angle in degrees, centered at 0 with range ±180°
     */
    private double getCurrentPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double degrees = (voltage / 3.3) * 360.0;

        // Convert from 0-360 range to ±180 range centered at encoder offset
        degrees -= RobotConstants.Shooter.ENCODER_OFFSET;

        // Normalize to [-180, 180]
        while (degrees > 180) degrees -= 360;
        while (degrees < -180) degrees += 360;

        return degrees;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        firstLoop = true;
    }

    @Override
    public void stop() {
        robot.turretServo.setPower(0);
    }
}
