package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.TurretConstants;

/**
 * Turret PIDF Tuning OpMode - Simplified version using Panels dashboard.
 *
 * Adjust values directly in the Panels dashboard:
 * - TurretConstants.TURRET_PID (PIDCoefficients object with p, i, d)
 * - TurretConstants.TURRET_TUNING_TARGET for target angle (turret degrees)
 *
 * Values update live without needing to restart the OpMode.
 *
 * Safety controls still on gamepad:
 * - X: Emergency stop
 * - A: Reset to center (0°)
 */
@TeleOp(name = "TurretPIDF", group = "Tests")
public class TurretPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;

    // Position tracking across ±180 boundary
    private double lastRawPosition = 0;
    private double cumulativePosition = 0;
    private int rotationCount = 0;

    // PID calculation variables
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;
    private boolean firstLoop = true;

    // Emergency stop flag
    private boolean emergencyStop = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        // Initialize position tracking (all in servo degrees)
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Normalize to [-180, 180]
        while (rawDegrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) rawDegrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (rawDegrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) rawDegrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        lastRawPosition = rawDegrees;
        cumulativePosition = rawDegrees;
        rotationCount = 0;

        telemetry.addData("Status", "Initialized - Adjust values in Panels dashboard");
        telemetry.addData("Controls", "X=Emergency Stop, A=Reset to Center");
        telemetry.addLine("Adjust TURRET_PID and TURRET_TUNING_TARGET in dashboard");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTime = System.nanoTime() / 1e9;
        firstLoop = true;
        emergencyStop = false;
    }

    @Override
    public void loop() {
        // Refresh values from Panels dashboard
        PanelsConfigurables.INSTANCE.refreshClass(RobotConstants.class);

        // EMERGENCY STOP - X button
        if (gamepad1.x) {
            emergencyStop = true;
            robot.turretServo.setPower(0);
        }

        // Reset from emergency stop - A button
        if (gamepad1.aWasPressed()) {
            emergencyStop = false;
            TurretConstants.TURRET_TUNING_TARGET = 0;
            resetPID();
        }

        if (emergencyStop) {
            robot.turretServo.setPower(0);
            telemetry.addLine("====== EMERGENCY STOP ======");
            telemetry.addLine("Press A to reset to center");
            telemetry.update();
            return;
        }

        // Get target from RobotConstants (turret degrees)
        double targetPositionTurret = TurretConstants.TURRET_TUNING_TARGET;

        // SAFETY: Clamp target position to hardware limits
        targetPositionTurret = Math.max(TurretConstants.TURRET_MIN_ANGLE,
                                        Math.min(TurretConstants.TURRET_MAX_ANGLE, targetPositionTurret));

        // Convert to servo degrees
        double targetPositionServo = targetPositionTurret * TurretConstants.GEAR_RATIO;

        // Get current position (in servo degrees)
        double currentServoPosition = getCurrentPosition();

        // Calculate error in servo degrees (NO wrapping - always unwind to true position)
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
        if (dt > RobotConstants.PID.DT_MAX || dt < RobotConstants.PID.DT_MIN) {
            dt = RobotConstants.PID.DT_DEFAULT;
        }

        lastTime = currentTime;

        // Get PID coefficients from RobotConstants
        double P = TurretConstants.TURRET_PID.p;
        double I = TurretConstants.TURRET_PID.i;
        double D = TurretConstants.TURRET_PID.d;

        // Update integral with anti-windup
        integral += error * dt;
        integral = Math.max(RobotConstants.PID.INTEGRAL_CLAMP_MIN, Math.min(RobotConstants.PID.INTEGRAL_CLAMP_MAX, integral));

        // Calculate derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Calculate PID output
        double output = P * error + I * integral + D * derivative;

        // Clamp output
        double currentTurretPosition = currentServoPosition / TurretConstants.GEAR_RATIO;
        boolean nearCWLimit = currentServoPosition > (TurretConstants.TURRET_MAX_ANGLE * TurretConstants.GEAR_RATIO - TurretConstants.TURRET_LIMIT_MARGIN);
        boolean nearCCWLimit = currentServoPosition < (TurretConstants.TURRET_MIN_ANGLE * TurretConstants.GEAR_RATIO + TurretConstants.TURRET_LIMIT_MARGIN);

        double maxPower = (nearCWLimit || nearCCWLimit) ? TurretConstants.TURRET_POWER_LIMIT_NEAR_EDGE : TurretConstants.TURRET_POWER_LIMIT_NORMAL;
        output = Math.max(-maxPower, Math.min(maxPower, output));

        // Set motor power
        robot.turretServo.setPower(output);

        // Telemetry - Safety warnings first
        if (nearCWLimit) {
            telemetry.addLine("WARNING: Approaching CW limit!");
        } else if (nearCCWLimit) {
            telemetry.addLine("WARNING: Approaching CCW limit!");
        }

        telemetry.addData("Target Position", "%.1f° turret (%.1f° servo)", targetPositionTurret, targetPositionServo);
        telemetry.addData("Current Position", "%.1f° turret (%.1f° servo)", currentTurretPosition, currentServoPosition);
        telemetry.addData("Error", "%.2f° servo", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P", "%.5f", P);
        telemetry.addData("I", "%.5f", I);
        telemetry.addData("D", "%.5f", D);
        telemetry.addLine("-----------------------------");
        telemetry.addData("PID Output", "%.3f", output);
        telemetry.addData("Motor Power", "%.3f", robot.turretServo.getPower());
        telemetry.addData("Integral", "%.3f", integral);
        telemetry.addData("Derivative", "%.3f", derivative);
        telemetry.addData("Raw Voltage", "%.3fV", robot.turretEncoder.getVoltage());
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Adjust values in Panels dashboard");
        telemetry.addLine("X=Emergency Stop, A=Reset to Center");
        telemetry.update();
    }

    /**
     * Gets the current servo position from the encoder
     * Tracks cumulative position across ±180° boundary to support full rotation range
     */
    private double getCurrentPosition() {
        double voltage = robot.turretEncoder.getVoltage();
        double rawDegrees = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Normalize to [-180, 180]
        while (rawDegrees > RobotConstants.Encoder.ANGLE_UPPER_BOUND) rawDegrees -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (rawDegrees < RobotConstants.Encoder.ANGLE_LOWER_BOUND) rawDegrees += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        // Track boundary crossings to maintain cumulative position
        double delta = rawDegrees - lastRawPosition;

        // Detect crossing from +180 to -180 (clockwise)
        if (delta < RobotConstants.Encoder.ANGLE_LOWER_BOUND) {
            rotationCount++;
        }
        // Detect crossing from -180 to +180 (counter-clockwise)
        else if (delta > RobotConstants.Encoder.ANGLE_UPPER_BOUND) {
            rotationCount--;
        }

        lastRawPosition = rawDegrees;
        cumulativePosition = rawDegrees + (rotationCount * RobotConstants.Encoder.FULL_ROTATION_DEGREES);

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
