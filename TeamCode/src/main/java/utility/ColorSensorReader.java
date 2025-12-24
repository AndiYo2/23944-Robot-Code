package utility;

import com.qualcomm.robotcore.hardware.ColorSensor;
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
     * @return The detected ball color (Purple, Green, or None)
     */
    public BallColor getBallColor() {
        // Purple ball: High red, medium green, high blue
        if (red > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[0] &&
            green > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[1] &&
            blue > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[2])
            return BallColor.Purple;

        // Green ball: Low red, high green, low blue
        else if (red < RobotConstants.ColorSensor.GREEN_THRESHOLDS[0] &&
                 green > RobotConstants.ColorSensor.GREEN_THRESHOLDS[1] &&
                 blue < RobotConstants.ColorSensor.GREEN_THRESHOLDS[2])
            return BallColor.Green;

        return BallColor.None;
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
