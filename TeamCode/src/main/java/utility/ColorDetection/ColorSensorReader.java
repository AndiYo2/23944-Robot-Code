package utility.ColorDetection;

import com.qualcomm.robotcore.hardware.ColorSensor;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;

/**
 * ColorSensorReader - A lightweight utility for reading color sensors
 *
 * Unlike ColorSensorSubsytem, this is a simple utility class that doesn't use
 * static fields, allowing you to create multiple instances for different sensors.
 *
 * Usage:
 *   ColorSensorReader sensor = new ColorSensorReader(hardwareMap.get(ColorSensor.class, "sensor1"));
 *   sensor.refreshScan();
 *   BallColor color = sensor.getBallColor();
 */
public class ColorSensorReader {
    private final ColorSensor colorSensor;
    private double red, green, blue, alpha;
    private BallColor lastDetectedColor = BallColor.None;

    public ColorSensorReader(ColorSensor sensor) {
        this.colorSensor = sensor;
        refreshScan();
    }

    /**
     * Updates the color readings from the sensor
     * Call this before checking ball color
     */
    public void refreshScan() {
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        alpha = colorSensor.alpha();
    }

    /**
     * Determines the ball color based on current sensor readings
     * Uses alpha channel to detect ball presence, then RGB ratios to identify color
     *
     * Logic:
     * 1. Check alpha to detect ball presence
     * 2. Use green-to-red ratio for color determination:
     *    - Green ball: green is significantly higher than red (ratio > 1.5)
     *    - Purple ball: red is higher or similar to green
     *
     * This ratio-based approach is more robust than fixed thresholds
     * because it works across different lighting conditions.
     *
     * @return The detected ball color (Purple, Green, or None)
     */
    public BallColor getBallColor() {
        // First check if a ball is present using alpha channel
        if (alpha <= RobotConstants.ColorSensor.ALPHA_THRESHOLD) {
            return BallColor.None;
        }

        // Ball is present - determine color using ratio
        // Avoid division by zero
        if (red < 10) {
            // Very low red, likely green ball
            return (green > RobotConstants.ColorSensor.GREEN_THRESHOLD) ? BallColor.Green : BallColor.None;
        }

        // Green ball: green must be higher than red AND exceed threshold
        // Purple ball: red is higher or similar to green
        if (green > red && green > RobotConstants.ColorSensor.GREEN_THRESHOLD) {
            return BallColor.Green;
        }

        // Otherwise, it's a purple ball
        return BallColor.Purple;
    }

    /**
     * Detects if a ball just entered the sensor area
     * Checks for transition from None to a color
     * @return true if a ball just entered
     */
    public boolean ballJustEntered() {
        BallColor current = getBallColor();
        boolean entered = (lastDetectedColor == BallColor.None && current != BallColor.None);
        lastDetectedColor = current;
        return entered;
    }

    /**
     * Resets the ball detection state
     * Call this when you know the sensor area is clear
     */
    public void reset() {
        lastDetectedColor = BallColor.None;
    }

    // Individual color getters
    public double getRed() { return red; }
    public double getGreen() { return green; }
    public double getBlue() { return blue; }
    public double getAlpha() { return alpha; }

    /**
     * Gets a formatted string of current color values
     * Useful for telemetry/debugging
     */
    public String getColorDataString() {
        return String.format("R=%.0f G=%.0f B=%.0f A=%.0f", red, green, blue, alpha);
    }
}
