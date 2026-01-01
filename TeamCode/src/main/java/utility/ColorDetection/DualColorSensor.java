package utility.ColorDetection;

import com.qualcomm.robotcore.hardware.ColorSensor;
import utility.RobotConstants.Enums.BallColor;

/**
 * DualColorSensor - Combines readings from two color sensors
 *
 * This is useful when you have two sensors offset from each other to avoid
 * missing the ball due to holes or gaps. The dual sensor approach increases
 * detection reliability.
 *
 * Detection Logic:
 * - If EITHER sensor detects a color, it's considered detected
 * - This prevents holes in the ball from causing false negatives
 *
 * Usage:
 *   DualColorSensor spindexerSensor = new DualColorSensor(sensor1, sensor2);
 *   spindexerSensor.refreshScan();
 *   BallColor color = spindexerSensor.getBallColor();
 */
public class DualColorSensor {
    private final ColorSensorReader sensor1;
    private final ColorSensorReader sensor2;
    private BallColor lastDetectedColor = BallColor.None;

    /**
     * Creates a dual sensor from two hardware ColorSensors
     */
    public DualColorSensor(ColorSensor sensor1, ColorSensor sensor2) {
        this.sensor1 = new ColorSensorReader(sensor1);
        this.sensor2 = new ColorSensorReader(sensor2);
    }

    /**
     * Creates a dual sensor from two ColorSensorReaders
     * Useful if you already have ColorSensorReader instances
     */
    public DualColorSensor(ColorSensorReader sensor1, ColorSensorReader sensor2) {
        this.sensor1 = sensor1;
        this.sensor2 = sensor2;
    }

    /**
     * Updates readings from both sensors
     * Call this before checking ball color
     */
    public void refreshScan() {
        sensor1.refreshScan();
        sensor2.refreshScan();
    }

    /**
     * Gets the ball color using readings from both sensors
     *
     * Strategy: Prioritize the sensor with HIGHER ALPHA (stronger signal).
     * - Compare alpha values from both sensors
     * - Use the color from the sensor with higher alpha (better ball detection)
     * - If both have low alpha, return None
     *
     * This alpha-based prioritization is more reliable than color prioritization
     * because a higher alpha indicates the ball is closer to that sensor or more
     * clearly detected, reducing false color readings.
     *
     * @return The detected ball color (Purple, Green, or None)
     */
    public BallColor getBallColor() {
        BallColor color1 = sensor1.getBallColor();
        BallColor color2 = sensor2.getBallColor();

        double alpha1 = sensor1.getAlpha();
        double alpha2 = sensor2.getAlpha();

        // If neither sensor detects a ball, return None
        if (color1 == BallColor.None && color2 == BallColor.None) {
            return BallColor.None;
        }

        // If only one sensor detects a ball, use that sensor's color
        if (color1 == BallColor.None) return color2;
        if (color2 == BallColor.None) return color1;

        // Both sensors detect a ball - prioritize the one with higher alpha (stronger signal)
        // This prevents weak/uncertain readings from overriding strong clear readings
        if (alpha1 > alpha2) {
            return color1;
        } else {
            return color2;
        }
    }

    /**
     * Alternative strategy: Require BOTH sensors to agree
     * More conservative - reduces false positives but may miss some detections
     *
     * @return The ball color if both sensors agree, None otherwise
     */
    public BallColor getBallColorConservative() {
        BallColor color1 = sensor1.getBallColor();
        BallColor color2 = sensor2.getBallColor();

        // Only return a color if both sensors agree
        if (color1 == color2) {
            return color1;
        }

        // Sensors disagree - return None to be safe
        return BallColor.None;
    }

    /**
     * Detects if a ball just entered the sensor area
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
     */
    public void reset() {
        sensor1.reset();
        sensor2.reset();
        lastDetectedColor = BallColor.None;
    }

    // Access to individual sensors for debugging
    public ColorSensorReader getSensor1() { return sensor1; }
    public ColorSensorReader getSensor2() { return sensor2; }

    /**
     * Gets formatted telemetry string showing both sensors
     */
    public String getTelemetryString() {
        double alpha1 = sensor1.getAlpha();
        double alpha2 = sensor2.getAlpha();
        String priority = (alpha1 > alpha2) ? "S1" : "S2";

        return String.format("S1: %s (α=%.0f) | S2: %s (α=%.0f) | Combined: %s [%s]",
                sensor1.getBallColor(), alpha1,
                sensor2.getBallColor(), alpha2,
                getBallColor(), priority);
    }

    /**
     * Gets detailed color data from both sensors
     */
    public String getDetailedColorData() {
        return String.format("Sensor1: %s\nSensor2: %s",
                sensor1.getColorDataString(),
                sensor2.getColorDataString());
    }

    /**
     * Get which sensor is currently being prioritized (has higher alpha)
     * @return 1 if sensor1 is prioritized, 2 if sensor2 is prioritized
     */
    public int getPrioritizedSensor() {
        return (sensor1.getAlpha() > sensor2.getAlpha()) ? 1 : 2;
    }
}
