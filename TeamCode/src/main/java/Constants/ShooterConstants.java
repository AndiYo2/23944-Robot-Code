package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Shooter flywheel and velocity constants.
 */
@Configurable
public class ShooterConstants {
    // ==================== FLIPPER POSITIONS ====================
    public static double FLIPPER_POSITION_EXTENDED = 0.4;  // Push ball into flywheel
    public static double FLIPPER_POSITION_RETRACT = 0.15;   // Ready position
    public static double FLICK_TIME = 0.2;                 // Seconds to hold extended

    // ==================== VELOCITY CONSTANTS ====================
    public static double DEFAULT_VELOCITY = 2200.0;           // Default/initial flywheel velocity
    public static double FALLBACK_VELOCITY = 2200.0;          // Fallback when lookup fails
    public static double DEFAULT_DISTANCE = 100.0;            // Default distance when turret unavailable

    // ==================== HOOD SERVO CONFIGURATION ====================
    // Hood servo: 0-355 deg physical range (Axon servo)
    // Hood angle: 0 = vertical, 90 = horizontal
    // Usable range: 30-63 degrees
    public static double HOOD_SERVO_CENTER_POSITION = 0.0;    // Servo position at 0 deg hood
    public static double HOOD_SERVO_DEGREES_PER_UNIT = 355.0; // Axon servo full range
    public static double HOOD_GEAR_RATIO = 1.0;               // Tune this if geared
    public static double HOOD_MIN_ANGLE = 30.0;               // Minimum hood angle (degrees)
    public static double HOOD_MAX_ANGLE = 63.0;               // Maximum hood angle (degrees)
    public static double HOOD_DEFAULT_ANGLE = 45.0;           // Default hood angle when no lookup
    public static double HOOD_SERVO_AT_MIN_ANGLE = .88;       // Servo position at 30 degrees (min angle)
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.26;      // Servo position at 63 degrees (max angle)

    // ==================== INTERPLUT DATA ====================
    // Format: {distance_inches, value} - MUST be sorted by distance ascending
    // Single LUT used by both alliances

    // Velocity LUT - Distance (inches) -> Velocity (ticks/sec)
    public static double[][] VELOCITY_DATA = {
        {0, 1720},
        {42, 1720},
        {55, 1820},
        {66, 1870},
        {80, 2020},
        {90, 2120},
        {100, 2200},
        {113, 2470},
        {128.6, 2570},
        {140.5, 2670},
        {146, 2720},
        {150, 2820},
        {300, 2820}
    };

    // Hood Angle LUT - Distance (inches) -> Hood Angle (degrees, 0=vertical, 90=horizontal)
    public static double[][] HOOD_DATA = {
        {0, 30},
        {42, 30},
        {55, 34},
        {66, 40},
        {80, 45},
        {90, 45.5},
        {100, 46},
        {113, 51},
        {128.6, 52},
        {140.5, 52.5},
        {146, 54},
        {150, 55},
        {300, 55}
    };

    /**
     * Shooter tuning mode for manual control of velocity and hood angle.
     * Enable TUNING_MODE to override automatic distance-based calculations.
     * Use Panels to adjust values in real-time and record them for the LUT.
     */
    @Configurable
    public static class ShooterTuning {
        /** Enable to override automatic velocity/hood angle with manual values */
        public static boolean TUNING_MODE = false;

        /** Manual velocity setting (ticks/sec) - only used when TUNING_MODE is true */
        public static double TUNING_VELOCITY = 2200.0;

        /** Manual hood angle setting (degrees) - only used when TUNING_MODE is true */
        public static double TUNING_HOOD_ANGLE = 45.0;
    }

    // ==================== FEEDFORWARD VELOCITY CONTROL ====================
    // References ShooterFeedforwardConstants for single source of truth.
    // See ShooterFeedforwardConstants for tuning documentation.

    /** Static friction compensation - references canonical source */
    public static double kS = ShooterFeedforwardConstants.kS;

    /** Velocity gain - references canonical source */
    public static double kV = ShooterFeedforwardConstants.kV;

    /** Acceleration gain - references canonical source */
    public static double kA = ShooterFeedforwardConstants.kA;

    /** Proportional gain - references canonical source */
    public static double VELOCITY_kP = ShooterFeedforwardConstants.kP;

    /** Integral gain - references canonical source */
    public static double VELOCITY_kI = ShooterFeedforwardConstants.kI;

    /** Derivative gain - references canonical source */
    public static double VELOCITY_kD = ShooterFeedforwardConstants.kD;

    /** Maximum integral accumulation - references canonical source */
    public static double INTEGRAL_MAX = ShooterFeedforwardConstants.INTEGRAL_MAX;

    /**
     * Maximum acceleration demand (ticks/sec^2).
     * Physical limit based on motor torque and flywheel inertia.
     */
    public static double MAX_ACCELERATION = 15000.0;

    /** Velocity tolerance - references canonical source */
    public static double VELOCITY_TOLERANCE = ShooterFeedforwardConstants.VELOCITY_TOLERANCE;
}
