package utility;

import utility.RobotConstants.Enums.BallColor;

/**
 * SmartColorSensorManager - Efficient color sensor management
 *
 * This manager reduces CPU and power usage by:
 * 1. Only reading sensors when needed
 * 2. Throttling sensor refresh rate
 * 3. Caching recent readings
 *
 * Usage:
 *   manager.update();  // Call once per loop
 *   BallColor color = manager.getSpindexerColor();
 */
public class SmartColorSensorManager {
    private final DualColorSensor spindexerSensor;
    private final DualColorSensor shooterSensor;

    // Cached values
    private BallColor lastSpindexerColor = BallColor.None;
    private BallColor lastShooterColor = BallColor.None;

    // Throttling
    private long lastSpindexerReadTime = 0;
    private long lastShooterReadTime = 0;
    private static final long MIN_READ_INTERVAL_MS = 50; // Read at most every 50ms (20Hz)

    // State tracking
    private boolean spindexerEnabled = true;
    private boolean shooterEnabled = false;  // Start disabled to save power

    public SmartColorSensorManager(DualColorSensor spindexerSensor, DualColorSensor shooterSensor) {
        this.spindexerSensor = spindexerSensor;
        this.shooterSensor = shooterSensor;
    }

    /**
     * Call this once per loop in your TeleOp/Auto
     * Only reads sensors that are enabled and throttles read rate
     */
    public void update() {
        long currentTime = System.currentTimeMillis();

        // Only read spindexer sensor if enabled and enough time has passed
        if (spindexerEnabled && (currentTime - lastSpindexerReadTime) >= MIN_READ_INTERVAL_MS) {
            spindexerSensor.refreshScan();
            lastSpindexerColor = spindexerSensor.getBallColor();
            lastSpindexerReadTime = currentTime;
        }

        // Only read shooter sensor if enabled and enough time has passed
        if (shooterEnabled && (currentTime - lastShooterReadTime) >= MIN_READ_INTERVAL_MS) {
            shooterSensor.refreshScan();
            lastShooterColor = shooterSensor.getBallColor();
            lastShooterReadTime = currentTime;
        }
    }

    /**
     * Enable/disable spindexer sensor to save power
     */
    public void setSpindexerEnabled(boolean enabled) {
        this.spindexerEnabled = enabled;
        if (!enabled) {
            lastSpindexerColor = BallColor.None;
        }
    }

    /**
     * Enable/disable shooter sensor to save power
     */
    public void setShooterEnabled(boolean enabled) {
        this.shooterEnabled = enabled;
        if (!enabled) {
            lastShooterColor = BallColor.None;
        }
    }

    /**
     * Get cached spindexer color (no I2C read)
     */
    public BallColor getSpindexerColor() {
        return lastSpindexerColor;
    }

    /**
     * Get cached shooter color (no I2C read)
     */
    public BallColor getShooterColor() {
        return lastShooterColor;
    }

    /**
     * Force an immediate sensor refresh (bypasses throttling)
     * Use sparingly - defeats the purpose of smart management
     */
    public void forceRefreshSpindexer() {
        spindexerSensor.refreshScan();
        lastSpindexerColor = spindexerSensor.getBallColor();
        lastSpindexerReadTime = System.currentTimeMillis();
    }

    /**
     * Check if a ball just entered the spindexer
     */
    public boolean spindexerBallJustEntered() {
        return spindexerSensor.ballJustEntered();
    }

    /**
     * Get telemetry string for debugging
     */
    public String getTelemetryString() {
        return String.format("Spindexer: %s (%s) | Shooter: %s (%s)",
                lastSpindexerColor,
                spindexerEnabled ? "ON" : "OFF",
                lastShooterColor,
                shooterEnabled ? "ON" : "OFF");
    }

    /**
     * Access underlying sensors for advanced usage
     */
    public DualColorSensor getSpindexerSensor() {
        return spindexerSensor;
    }

    public DualColorSensor getShooterSensor() {
        return shooterSensor;
    }
}
