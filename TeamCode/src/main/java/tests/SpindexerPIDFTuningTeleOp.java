package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import utility.RobotConstants;
import utility.RobotHardware;

@TeleOp(name = "SpindexerPIDF", group = "Tests")
public class SpindexerPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;

    // Target positions (in degrees)
    private double targetPosition = 62;
    private double[] targetPositions = {62, 182, 302};
    private int targetIndex = 0;

    // PID coefficients - INCREASED starting values
    private double P = RobotConstants.Spindexer.SPINDEXER_PID.p;  // Increased from 0.0001
    private double I = RobotConstants.Spindexer.SPINDEXER_PID.i;
    private double D = RobotConstants.Spindexer.SPINDEXER_PID.d; // Added small D term

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
        telemetry.addData("Instructions", "");
        telemetry.addLine("Y: Cycle Target Position");
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
        // Handle input for changing target position
        if (gamepad1.yWasPressed()) {
            targetIndex = (targetIndex + 1) % targetPositions.length;
            targetPosition = targetPositions[targetIndex];
            resetPID();
        }

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
        robot.spindexerServo.setPower(output);

        // Telemetry
        telemetry.addData("Target Position", "%.1f°", targetPosition);
        telemetry.addData("Current Position", "%.1f°", currentPosition);
        telemetry.addData("Error", "%.2f°", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("P", "%.5f (D-Pad U/D)", P);
        telemetry.addData("I", "%.5f (Bumpers)", I);
        telemetry.addData("D", "%.5f (D-Pad L/R)", D);
        telemetry.addData("Step Size", "%.5f (B)", stepSizes[stepIndex]);
        telemetry.addLine("-----------------------------");
        telemetry.addData("PID Output", "%.3f", output);
        telemetry.addData("Motor Power", "%.3f", robot.spindexerServo.getPower());
        telemetry.addData("Integral", "%.3f", integral);
        telemetry.addData("Derivative", "%.3f", derivative);
        telemetry.addData("dt", "%.4f", dt);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Press Y to cycle target position");
        telemetry.addLine("Current PID Coefficients:");
        telemetry.addData("", "new PIDCoefficients(%.5f, %.5f, %.5f)", P, I, D);
        telemetry.update();
    }

    private double getCurrentPosition() {
        double voltage = robot.spindexerEncoder.getVoltage();
        double position = (voltage / 3.3) * 360;
        return position;
    }

    private void resetPID() {
        integral = 0;
        lastError = 0;
        firstLoop = true;
    }

    @Override
    public void stop() {
        robot.spindexerServo.setPower(0);
    }
}