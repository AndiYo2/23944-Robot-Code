package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Shooter flywheel and velocity constants.
 */
@Configurable
public class ShooterConstants {
    // ==================== FLIPPER POSITIONS ====================
    public static double FLIPPER_POSITION_EXTENDED = 0.35;  // Push ball into flywheel
    public static double FLIPPER_POSITION_RETRACT = 0.15;   // Ready position
    public static double FLICK_TIME = 0.15;                 // Seconds to hold extended

    // ==================== FLYWHEEL PIDF ====================
    // Velocity control for flywheel motors (ticks/sec)
    public static double SHOOTER_P = 40;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 10.2;

    // Target velocity for shooter tuning mode (ticks/sec)
    // Adjust this via configurables to test shooter PIDF at different speeds
    public static double SHOOTER_TUNING_VELOCITY = 2200.0;

    // ==================== VELOCITY CONSTANTS ====================
    public static double DEFAULT_VELOCITY = 2200.0;           // Default/initial flywheel velocity
    public static double FALLBACK_VELOCITY = 2200.0;          // Fallback when lookup fails

    // ==================== VELOCITY LOOKUP TABLES ====================
    // Zone boundary (inches) - below uses front table, above uses back table
    public static double BLUE_ZONE_BOUNDARY = 105.0;
    public static double RED_ZONE_BOUNDARY = 100.0;

    // Blue Front Zone: distance (inches) to velocity (ticks/sec)
    public static double[][] BLUE_FRONT_LOOKUP = {
        {53, 1900},
        {78, 2000},
        {80, 1950},
        {94, 2030},
        {98, 2100},
        {100, 2100}
    };

    // Blue Back Zone
    public static double[][] BLUE_BACK_LOOKUP = {
        {140, 2600},
        {144, 2600},
        {148, 2650},
        {154, 2700}
    };

    // Red Front Zone
    public static double[][] RED_FRONT_LOOKUP = {
        {48, 2000},
        {52, 2000},
        {80, 2100},
        {85, 2100}
    };

    // Red Back Zone
    public static double[][] RED_BACK_LOOKUP = {
        {134, 2500},
        {138, 2520},
        {148, 2650}
    };

    /**
     * Shooter PIDF values in a separate class for independent live tuning via Panels.
     * Refreshing this class won't affect turret PID or other Shooter settings.
     */
    @Configurable
    public static class ShooterPIDF {
        public static double P = 16;
        public static double I = 0.0;
        public static double D = 0.0;
        public static double F = 10;
        public static double TUNING_VELOCITY = 2625.0;
    }
}
