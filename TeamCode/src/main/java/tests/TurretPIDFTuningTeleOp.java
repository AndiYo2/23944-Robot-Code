package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import utility.RobotConstants;
import utility.RobotHardware;

@TeleOp(name = "TurretPIDF", group = "Tests")
public class TurretPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;

    // Target positions in TURRET degrees (will be converted to servo degrees internally)
    // Turret range: 45° CW, -60° CCW → Servo range: 270° CW, -360° CCW (6:1 ratio)
    // Test sequence: 0→15→30→45→0 (CW), then 0→-15→-30→-45→-60→0 (CCW)
    private double targetPositionTurret = 0; // Input in turret degrees
    private double targetPositionServo = 0;  // Converted to servo degrees for PID
    private double[] targetPositionsTurret = {
        0,      // Start
        15, 30, 45,  // Clockwise to 45° turret (270° servo)
        0,      // Unwind back to center
        -15, -30, -45, -60,  // Counter-clockwise to -60° turret (-360° servo)
        0       // Unwind back to center
    };
    private int targetIndex = 0;

    // Manual angle adjustment (for fine control)
    private boolean manualMode = false;

    // Position tracking across ±180 boundary
    private double lastRawPosition = 0;
    private double cumulativePosition = 0;
    private int rotationCount = 0;

    // PID coefficients - Tuned values
    private double P = 0.013;
    private double I = 0;
    private double D = 0.00030;

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

        // Initialize position tracking (all in servo degrees)
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;
        rawDegrees -= RobotConstants.Shooter.ENCODER_OFFSET;
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        lastRawPosition = rawDegrees;
        cumulativePosition = rawDegrees;
        rotationCount = 0;

        // Initialize target in servo degrees
        targetPositionServo = cumulativePosition;
        targetPositionTurret = 0; // Start at turret center

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Hard Limits", "Turret: 45° / -60° | Servo: 270° / -360°");
        telemetry.addData("Gear Ratio", "6:1 (servo → turret)");
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
        if (gamepad1.yWasPressed()) {
            targetIndex = (targetIndex + 1) % targetPositionsTurret.length;
            targetPositionTurret = targetPositionsTurret[targetIndex];
            targetPositionServo = targetPositionTurret * RobotConstants.Shooter.GEAR_RATIO;
            manualMode = false;
            resetPID();
        }

        // Handle input for resetting to center
        if (gamepad1.aWasPressed()) {
            targetPositionTurret = 0;
            targetPositionServo = 0;
            manualMode = false;
            resetPID();
        }

        // Manual angle adjustment with left stick (in turret degrees)
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            manualMode = true;
            targetPositionTurret += gamepad1.left_stick_x * 2.0; // 2 degrees per tick at full stick
        }

        // SAFETY: Clamp target position to asymmetric hard limits (45° CW, -60° CCW turret)
        targetPositionTurret = Math.max(-60.0, Math.min(45.0, targetPositionTurret));
        targetPositionServo = targetPositionTurret * RobotConstants.Shooter.GEAR_RATIO;

        // Handle input for changing step size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Handle P tuning
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
            P = Math.max(0, P);
        }

        // Handle D tuning
        if (gamepad1.dpadRightWasPressed()) {
            D += stepSizes[stepIndex];
        }
        if (gamepad1.dpadLeftWasPressed()) {
            D -= stepSizes[stepIndex];
            D = Math.max(0, D);
        }

        // Handle I tuning
        if (gamepad1.rightBumperWasPressed()) {
            I += stepSizes[stepIndex];
        }
        if (gamepad1.leftBumperWasPressed()) {
            I -= stepSizes[stepIndex];
            I = Math.max(0, I);
        }

        // Get current position (in servo degrees)
        double currentServoPosition = getCurrentPosition();

        // Calculate error in servo degrees (NO wrapping - always unwind to true position)
        // This ensures when returning to 0° from 45° turret (270° servo), it fully rotates back
        double error = targetPositionServo - currentServoPosition;

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

        // Check if approaching limits (servo: 270° CW, -360° CCW)
        double currentTurretPosition = currentServoPosition / RobotConstants.Shooter.GEAR_RATIO;
        boolean nearCWLimit = currentServoPosition > 240.0;  // Warn at 240° servo (40° turret)
        boolean nearCCWLimit = currentServoPosition < -300.0; // Warn at -300° servo (-50° turret)
        boolean atCWLimit = targetPositionTurret >= 45.0;
        boolean atCCWLimit = targetPositionTurret <= -60.0;

        // Telemetry - Safety warnings first
        if (atCWLimit) {
            telemetry.addLine("⚠⚠⚠ AT CW LIMIT (45° turret = 270° servo) ⚠⚠⚠");
        } else if (atCCWLimit) {
            telemetry.addLine("⚠⚠⚠ AT CCW LIMIT (-60° turret = -360° servo) ⚠⚠⚠");
        } else if (nearCWLimit) {
            telemetry.addLine("⚠ WARNING: Approaching CW limit!");
        } else if (nearCCWLimit) {
            telemetry.addLine("⚠ WARNING: Approaching CCW limit!");
        }

        telemetry.addData("Target Position", "%.1f° turret (%.1f° servo) %s",
                         targetPositionTurret, targetPositionServo, manualMode ? "(MANUAL)" : "");
        telemetry.addData("Current Position", "%.1f° turret (%.1f° servo)",
                         currentTurretPosition, currentServoPosition);
        telemetry.addData("Raw Position", "%.1f° (Rotations: %d)", lastRawPosition, rotationCount);
        telemetry.addData("Error", "%.2f° servo", error);
        telemetry.addData("Hard Limits", "Turret: 45° / -60° | Servo: 270° / -360°");
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
     * Gets the current servo position from the encoder
     * Tracks cumulative position across ±180° boundary to support full rotation range
     * @return Servo angle in degrees, cumulative position
     */
    private double getCurrentPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / 3.3) * 360.0;

        // Apply encoder offset
        rawDegrees -= RobotConstants.Shooter.ENCODER_OFFSET;

        // Normalize to [-180, 180]
        while (rawDegrees > 180) rawDegrees -= 360;
        while (rawDegrees < -180) rawDegrees += 360;

        // Track boundary crossings to maintain cumulative position
        double delta = rawDegrees - lastRawPosition;

        // Detect crossing from +180 to -180 (clockwise)
        if (delta < -180) {
            rotationCount++;
        }
        // Detect crossing from -180 to +180 (counter-clockwise)
        else if (delta > 180) {
            rotationCount--;
        }

        lastRawPosition = rawDegrees;
        cumulativePosition = rawDegrees + (rotationCount * 360);

        return cumulativePosition;
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
