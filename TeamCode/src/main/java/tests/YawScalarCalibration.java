package tests;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Yaw Scalar Calibration Tool
 *
 * This OpMode helps calibrate the Pinpoint's yaw scalar to correct heading drift.
 *
 * INSTRUCTIONS:
 * 1. Place robot on flat surface, mark starting orientation
 * 2. Press INIT - keep robot COMPLETELY STILL
 * 3. Press PLAY
 * 4. Manually rotate the robot EXACTLY 10 full rotations (3600 degrees) in ONE direction
 * 5. Return to exact starting orientation
 * 6. Press A to calculate the yaw scalar
 * 7. Update RobotConstants.Pinpoint.yawScalar with the calculated value
 */
@TeleOp(name = "Yaw Scalar Calibration", group = "Calibration")
public class YawScalarCalibration extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Tracking variables
    private double totalRotation = 0;
    private double lastHeading = 0;
    private boolean calibrationComplete = false;
    private double calculatedYawScalar = 1.0;

    // Target rotations for calibration
    private static final double TARGET_ROTATIONS = 10.0;
    private static final double TARGET_DEGREES = TARGET_ROTATIONS * 360.0; // 3600 degrees

    @Override
    public void runOpMode() {
        // Initialize Pinpoint directly (not using RobotHardware to avoid other subsystems)
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.INCH);

        // Set yaw scalar to 1.0 for calibration (no correction)
        pinpoint.setYawScalar(1.0);

        telemetry.addLine("=== YAW SCALAR CALIBRATION ===");
        telemetry.addLine("");
        telemetry.addLine("KEEP ROBOT COMPLETELY STILL!");
        telemetry.addLine("Calibrating IMU...");
        telemetry.update();

        // Reset and calibrate IMU
        pinpoint.resetPosAndIMU();
        sleep(500); // Extra time for calibration

        telemetry.addLine("=== YAW SCALAR CALIBRATION ===");
        telemetry.addLine("");
        telemetry.addLine("IMU Calibrated!");
        telemetry.addLine("");
        telemetry.addLine("INSTRUCTIONS:");
        telemetry.addLine("1. Mark robot's starting orientation");
        telemetry.addLine("2. Press PLAY");
        telemetry.addLine("3. Rotate robot 10 FULL rotations");
        telemetry.addLine("   (in ONE direction only)");
        telemetry.addLine("4. Return to EXACT starting position");
        telemetry.addLine("5. Press A to calculate yaw scalar");
        telemetry.addLine("");
        telemetry.addLine("Press PLAY when ready...");
        telemetry.update();

        waitForStart();

        // Initialize tracking
        pinpoint.update();
        lastHeading = pinpoint.getHeading(AngleUnit.DEGREES);
        totalRotation = 0;

        while (opModeIsActive()) {
            pinpoint.update();

            // Get current heading
            double currentHeading = pinpoint.getHeading(AngleUnit.DEGREES);

            // Calculate delta (handle wraparound)
            double delta = currentHeading - lastHeading;
            if (delta > 180) delta -= 360;
            if (delta < -180) delta += 360;

            // Accumulate total rotation
            totalRotation += delta;
            lastHeading = currentHeading;

            // Check for A button to complete calibration
            if (gamepad1.a && !calibrationComplete && Math.abs(totalRotation) > 100) {
                calibrationComplete = true;
                // Calculate yaw scalar: actual / reported
                calculatedYawScalar = TARGET_DEGREES / Math.abs(totalRotation);
            }

            // Reset with B button
            if (gamepad1.b) {
                totalRotation = 0;
                calibrationComplete = false;
                pinpoint.resetPosAndIMU();
                sleep(300);
                pinpoint.update();
                lastHeading = pinpoint.getHeading(AngleUnit.DEGREES);
            }

            // Display telemetry
            telemetry.addLine("=== YAW SCALAR CALIBRATION ===");
            telemetry.addLine("");

            if (!calibrationComplete) {
                telemetry.addLine("ROTATE THE ROBOT 10 FULL TIMES");
                telemetry.addLine("then return to starting position");
                telemetry.addLine("");
                telemetry.addData("Current Heading", "%.2f deg", currentHeading);
                telemetry.addData("Total Rotation", "%.2f deg", totalRotation);
                telemetry.addData("Rotations Completed", "%.2f / %.0f",
                    Math.abs(totalRotation) / 360.0, TARGET_ROTATIONS);
                telemetry.addLine("");

                // Progress bar
                double progress = Math.min(Math.abs(totalRotation) / TARGET_DEGREES, 1.0);
                int barLength = 20;
                int filledLength = (int) (progress * barLength);
                StringBuilder bar = new StringBuilder("[");
                for (int i = 0; i < barLength; i++) {
                    bar.append(i < filledLength ? "=" : " ");
                }
                bar.append("]");
                telemetry.addData("Progress", "%s %.0f%%", bar.toString(), progress * 100);
                telemetry.addLine("");

                if (Math.abs(totalRotation) >= TARGET_DEGREES * 0.9) {
                    telemetry.addLine(">>> Almost there! <<<");
                    telemetry.addLine("Return to starting position");
                    telemetry.addLine("then press A to calculate");
                }

                telemetry.addLine("");
                telemetry.addLine("Controls:");
                telemetry.addLine("  A = Calculate yaw scalar");
                telemetry.addLine("  B = Reset and start over");
            } else {
                telemetry.addLine("=== CALIBRATION COMPLETE ===");
                telemetry.addLine("");
                telemetry.addData("Target Rotation", "%.0f deg", TARGET_DEGREES);
                telemetry.addData("Reported Rotation", "%.2f deg", Math.abs(totalRotation));
                telemetry.addLine("");
                telemetry.addLine("=============================");
                telemetry.addData("NEW YAW SCALAR", "%.6f", calculatedYawScalar);
                telemetry.addLine("=============================");
                telemetry.addLine("");
                telemetry.addLine("Update RobotConstants.java:");
                telemetry.addData("yawScalar", "%.6f", calculatedYawScalar);
                telemetry.addLine("");

                // Interpretation
                if (calculatedYawScalar > 1.0) {
                    telemetry.addLine("Pinpoint under-reported rotation");
                    telemetry.addData("Drift", "%.2f deg per 360 deg",
                        360.0 - (360.0 / calculatedYawScalar));
                } else {
                    telemetry.addLine("Pinpoint over-reported rotation");
                    telemetry.addData("Drift", "%.2f deg per 360 deg",
                        (360.0 / calculatedYawScalar) - 360.0);
                }

                telemetry.addLine("");
                telemetry.addLine("Press B to reset and recalibrate");
            }

            telemetry.update();
        }
    }
}