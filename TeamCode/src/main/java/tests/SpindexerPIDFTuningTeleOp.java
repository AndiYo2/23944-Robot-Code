package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.SpindexerConstants;

/**
 * Spindexer PIDF Tuning OpMode - Simplified version using Panels dashboard.
 *
 * Adjust values directly in the Panels dashboard:
 * - SpindexerConstants.SPINDEXER_CW_P/I/D/F for CW PIDF coefficients
 * - SpindexerConstants.SPINDEXER_CCW_P/I/D/F for CCW PIDF coefficients
 * - SpindexerConstants.TUNING_TARGET_POSITION for target position (degrees)
 *
 * Values update live without needing to restart the OpMode.
 */
@TeleOp(name = "SpindexerPIDF", group = "Tests")
public class SpindexerPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;

    // PID calculation variables
    private double lastError = 0;
    private double integral = 0;
    private double lastTime = 0;
    private boolean firstLoop = true;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - Adjust values in Panels dashboard");
        telemetry.addLine("Adjust SPINDEXER_CW_P/I/D/F or SPINDEXER_CCW_P/I/D/F");
        telemetry.addLine("Adjust TUNING_TARGET_POSITION for target angle");
        telemetry.update();
    }

    @Override
    public void start() {
        lastTime = System.nanoTime() / 1e9;
        firstLoop = true;
    }

    @Override
    public void loop() {
        // Refresh values from Panels dashboard
        PanelsConfigurables.INSTANCE.refreshClass(RobotConstants.class);

        // Get current position
        double currentPosition = getCurrentPosition();
        double targetPosition = SpindexerConstants.TUNING_TARGET_POSITION;

        // Calculate error (shortest path)
        double error = targetPosition - currentPosition;
        if (error > RobotConstants.Encoder.ANGLE_UPPER_BOUND) error -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        if (error < RobotConstants.Encoder.ANGLE_LOWER_BOUND) error += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

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

        // Determine direction and select appropriate PIDF gains
        boolean isCW = error > 0;
        double P, I, D, F;
        if (isCW) {
            P = SpindexerConstants.SPINDEXER_CW_P;
            I = SpindexerConstants.SPINDEXER_CW_I;
            D = SpindexerConstants.SPINDEXER_CW_D;
            F = SpindexerConstants.SPINDEXER_CW_F;
        } else {
            P = SpindexerConstants.SPINDEXER_CCW_P;
            I = SpindexerConstants.SPINDEXER_CCW_I;
            D = SpindexerConstants.SPINDEXER_CCW_D;
            F = SpindexerConstants.SPINDEXER_CCW_F;
        }

        // Update integral with anti-windup
        integral += error * dt;
        integral = Math.max(RobotConstants.PID.INTEGRAL_CLAMP_MIN, Math.min(RobotConstants.PID.INTEGRAL_CLAMP_MAX, integral));

        // Calculate derivative
        double derivative = (error - lastError) / dt;
        lastError = error;

        // Calculate PIDF output
        double pidOutput = P * error + I * integral + D * derivative;
        double feedforward = F * Math.signum(error);
        double output = pidOutput + feedforward;

        // Clamp output
        output = Math.max(-1, Math.min(1, output));

        // Set motor power
        robot.spindexerServo.setPower(output);

        // Telemetry
        telemetry.addData("Target Position", "%.1f°", targetPosition);
        telemetry.addData("Current Position", "%.1f°", currentPosition);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addData("Direction", isCW ? "CW" : "CCW");
        telemetry.addLine("-----------------------------");
        telemetry.addData("Active P", "%.5f", P);
        telemetry.addData("Active I", "%.5f", I);
        telemetry.addData("Active D", "%.5f", D);
        telemetry.addData("Active F", "%.5f", F);
        telemetry.addLine("-----------------------------");
        telemetry.addData("PID Output", "%.3f", pidOutput);
        telemetry.addData("Feedforward", "%.3f", feedforward);
        telemetry.addData("Total Output", "%.3f", output);
        telemetry.addData("Integral", "%.3f", integral);
        telemetry.addData("Derivative", "%.3f", derivative);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Adjust values in Panels dashboard");
        telemetry.update();
    }

    private double getCurrentPosition() {
        double voltage = robot.spindexerEncoder.getVoltage();
        double position = (voltage / RobotConstants.Encoder.MAX_VOLTAGE) * RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        return position;
    }

    @Override
    public void stop() {
        robot.spindexerServo.setPower(0);
    }
}
