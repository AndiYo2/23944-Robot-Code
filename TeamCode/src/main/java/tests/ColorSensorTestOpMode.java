package tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;

/**
 * ColorSensorTestOpMode - Advanced color sensor testing and threshold tuning
 *
 * Features:
 * - Real-time RGB+Alpha values for both sensors
 * - Color detection based on RobotConstants thresholds
 * - Adjustable threshold scaling factors
 * - Green-to-red ratio analysis (for debugging)
 * - Distance measurement (if available)
 * - Step size adjustment for fine-tuning
 *
 * Controls:
 * - D-Pad Up/Down: Adjust PURPLE_THRESHOLD
 * - D-Pad Left/Right: Adjust GREEN_THRESHOLD
 * - Left Bumper/Right Bumper: Adjust ALPHA_THRESHOLD
 * - A Button: Cycle step size (100, 10, 1)
 * - X Button: Reset thresholds to defaults
 *
 * Detection Logic:
 * - Green detected if: (green > red) AND (green > GREEN_THRESHOLD) AND (alpha > ALPHA_THRESHOLD)
 * - Otherwise: Purple (if ball present) or None
 */
@TeleOp(name = "ColorSensorTestOpMode", group = "Tests")
public class ColorSensorTestOpMode extends OpMode {

    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;
    private NormalizedColorSensor normalizedSensor1;
    private NormalizedColorSensor normalizedSensor2;
    private DistanceSensor distanceSensor1;
    private DistanceSensor distanceSensor2;

    // Adjustable threshold scaling factors
    private double purpleThreshold = RobotConstants.ColorSensor.PURPLE_THRESHOLD;
    private double greenThreshold = RobotConstants.ColorSensor.GREEN_THRESHOLD;
    private double alphaThreshold = RobotConstants.ColorSensor.ALPHA_THRESHOLD;

    // Step sizes for threshold adjustment
    private double[] stepSizes = {100, 10, 1};
    private int stepIndex = 1; // Default to step size of 10

    // Sensor readings
    private double red1, green1, blue1, alpha1;
    private double red2, green2, blue2, alpha2;

    @Override
    public void init() {
        // Initialize color sensors
        colorSensor1 = hardwareMap.get(ColorSensor.class, "intakeSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "intakeSensor2");

        // Try to get normalized color sensor interface for additional features
        try {
            normalizedSensor1 = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor1");
            normalizedSensor2 = hardwareMap.get(NormalizedColorSensor.class, "intakeSensor2");
        } catch (Exception e) {
            normalizedSensor1 = null;
            normalizedSensor2 = null;
        }

        // Try to get distance sensor interface if available
        try {
            distanceSensor1 = hardwareMap.get(DistanceSensor.class, "intakeSensor1");
            distanceSensor2 = hardwareMap.get(DistanceSensor.class, "intakeSensor2");
        } catch (Exception e) {
            distanceSensor1 = null;
            distanceSensor2 = null;
        }

        // Set gain if normalized sensor is available (2x gain recommended for better sensitivity)
        if (normalizedSensor1 != null) {
            normalizedSensor1.setGain(2);
        }
        if (normalizedSensor2 != null) {
            normalizedSensor2.setGain(2);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Sensor 1", colorSensor1 != null ? "Connected" : "Not Found");
        telemetry.addData("Sensor 2", colorSensor2 != null ? "Connected" : "Not Found");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Handle threshold adjustments
        handleControls();

        // Read sensor values
        updateSensorReadings();

        // Detect colors using adjustable thresholds
        BallColor color1 = detectColor(red1, green1, blue1, alpha1);
        BallColor color2 = detectColor(red2, green2, blue2, alpha2);

        // Calculate ratios for analysis
        double ratio1 = (red1 > 10) ? green1 / red1 : 0;
        double ratio2 = (red2 > 10) ? green2 / red2 : 0;

        // Display telemetry
        displayTelemetry(color1, color2, ratio1, ratio2);
    }

    private void handleControls() {
        // Cycle step size with A button
        if (gamepad1.a) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // Reset thresholds to defaults with X button
        if (gamepad1.x) {
            purpleThreshold = RobotConstants.ColorSensor.PURPLE_THRESHOLD;
            greenThreshold = RobotConstants.ColorSensor.GREEN_THRESHOLD;
            alphaThreshold = RobotConstants.ColorSensor.ALPHA_THRESHOLD;
        }

        // Adjust PURPLE_THRESHOLD with D-Pad Up/Down
        if (gamepad1.dpad_up) {
            purpleThreshold += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_down) {
            purpleThreshold = Math.max(0, purpleThreshold - stepSizes[stepIndex]);
        }

        // Adjust GREEN_THRESHOLD with D-Pad Left/Right
        if (gamepad1.dpad_right) {
            greenThreshold += stepSizes[stepIndex];
        }
        if (gamepad1.dpad_left) {
            greenThreshold = Math.max(0, greenThreshold - stepSizes[stepIndex]);
        }

        // Adjust ALPHA_THRESHOLD with Bumpers
        if (gamepad1.right_bumper) {
            alphaThreshold += stepSizes[stepIndex];
        }
        if (gamepad1.left_bumper) {
            alphaThreshold = Math.max(0, alphaThreshold - stepSizes[stepIndex]);
        }
    }

    private void updateSensorReadings() {
        // Sensor 1 readings
        red1 = colorSensor1.red();
        green1 = colorSensor1.green();
        blue1 = colorSensor1.blue();
        alpha1 = colorSensor1.alpha();

        // Sensor 2 readings
        red2 = colorSensor2.red();
        green2 = colorSensor2.green();
        blue2 = colorSensor2.blue();
        alpha2 = colorSensor2.alpha();
    }

    /**
     * Color detection algorithm based on ColorSensorReader logic
     * Uses adjustable thresholds instead of constants
     *
     * Detection rule: Green detected if green > red AND green > threshold
     */
    private BallColor detectColor(double red, double green, double blue, double alpha) {
        // First check if a ball is present using alpha channel
        if (alpha <= alphaThreshold) {
            return BallColor.None;
        }

        // Ball is present - determine color
        if (red < 10) {
            // Very low red, likely green ball
            return (green > greenThreshold) ? BallColor.Green : BallColor.None;
        }

        // Green ball: green must be higher than red AND exceed threshold
        if (green > red && green > greenThreshold) {
            return BallColor.Green;
        }

        // Otherwise, it's a purple ball
        return BallColor.Purple;
    }

    private void displayTelemetry(BallColor color1, BallColor color2, double ratio1, double ratio2) {
        // Determine which sensor would be prioritized (higher alpha)
        String prioritizedSensor = (alpha1 > alpha2) ? "SENSOR 1" : "SENSOR 2";
        BallColor prioritizedColor = (alpha1 > alpha2) ? color1 : color2;

        // Header
        telemetry.addLine("=== COLOR SENSOR TEST ===");
        telemetry.addData("PRIORITIZED", "%s -> %s", prioritizedSensor, getColorString(prioritizedColor));
        telemetry.addLine();

        // Sensor 1 Data
        telemetry.addLine("--- SENSOR 1 (intakeSensor1) ---");
        telemetry.addData("Detected Color", getColorString(color1));
        telemetry.addData("Red", "%.0f", red1);
        telemetry.addData("Green", "%.0f", green1);
        telemetry.addData("Blue", "%.0f", blue1);
        telemetry.addData("Alpha", "%.0f (Ball present: %s)", alpha1, alpha1 > alphaThreshold ? "YES" : "NO");
        telemetry.addData("Green/Red Ratio", "%.2f", ratio1);

        // Add distance if available
        if (distanceSensor1 != null) {
            try {
                double distance = distanceSensor1.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance", "%.1f mm", distance);
            } catch (Exception e) {
                // Distance not available
            }
        }

        // Add normalized values if available
        if (normalizedSensor1 != null) {
            NormalizedRGBA colors = normalizedSensor1.getNormalizedColors();
            telemetry.addData("Normalized", "R=%.2f G=%.2f B=%.2f A=%.2f",
                colors.red, colors.green, colors.blue, colors.alpha);
        }

        telemetry.addLine();

        // Sensor 2 Data
        telemetry.addLine("--- SENSOR 2 (intakeSensor2) ---");
        telemetry.addData("Detected Color", getColorString(color2));
        telemetry.addData("Red", "%.0f", red2);
        telemetry.addData("Green", "%.0f", green2);
        telemetry.addData("Blue", "%.0f", blue2);
        telemetry.addData("Alpha", "%.0f (Ball present: %s)", alpha2, alpha2 > alphaThreshold ? "YES" : "NO");
        telemetry.addData("Green/Red Ratio", "%.2f", ratio2);

        // Add distance if available
        if (distanceSensor2 != null) {
            try {
                double distance = distanceSensor2.getDistance(DistanceUnit.MM);
                telemetry.addData("Distance", "%.1f mm", distance);
            } catch (Exception e) {
                // Distance not available
            }
        }

        // Add normalized values if available
        if (normalizedSensor2 != null) {
            NormalizedRGBA colors = normalizedSensor2.getNormalizedColors();
            telemetry.addData("Normalized", "R=%.2f G=%.2f B=%.2f A=%.2f",
                colors.red, colors.green, colors.blue, colors.alpha);
        }

        telemetry.addLine();

        // Threshold Settings
        telemetry.addLine("=== THRESHOLD SETTINGS ===");
        telemetry.addData("Purple Threshold", "%.0f (D-Pad U/D)", purpleThreshold);
        telemetry.addData("Green Threshold", "%.0f (D-Pad L/R)", greenThreshold);
        telemetry.addData("Alpha Threshold", "%.0f (Bumpers)", alphaThreshold);
        telemetry.addData("Step Size", "%.0f (A Button)", stepSizes[stepIndex]);
        telemetry.addLine();
        telemetry.addData("Reset to Defaults", "X Button");

        telemetry.update();
    }

    private String getColorString(BallColor color) {
        switch (color) {
            case Purple: return "PURPLE";
            case Green: return "GREEN";
            case None: return "NONE";
            default: return "UNKNOWN";
        }
    }
}