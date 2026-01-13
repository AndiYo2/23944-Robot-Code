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

    // ==================== FEEDFORWARD VELOCITY CONTROL ====================
    // Custom feedforward + PID for fast velocity recovery after shots.
    // All values tunable via panels.

    /**
     * Static friction compensation (kS).
     * Minimum power to overcome friction and start flywheel moving.
     * Find by slowly increasing power from 0 until flywheel starts spinning.
     */
    public static double kS = 0.05;

    /**
     * Velocity gain (kV).
     * Maps target velocity to motor power. Units: power per (tick/sec).
     * Find by running motor at power=1.0, measure max velocity, kV = 1.0/maxVelocity.
     * Example: max velocity 2800 ticks/sec -> kV = 0.000357
     */
    public static double kV = 0.00035;

    /**
     * Acceleration gain (kA).
     * Compensates for flywheel inertia during recovery. Critical for fast recovery.
     * Start small (0.00001), increase until recovery is fast without overshoot.
     */
    public static double kA = 0.00001;

    /**
     * Proportional gain for velocity error correction.
     * Reacts to current error. Tune after feedforward is working.
     */
    public static double VELOCITY_kP = 0.0001;

    /**
     * Integral gain for steady-state error elimination.
     * Accumulates error over time. Start very small (0.0001).
     */
    public static double VELOCITY_kI = 0.0002;

    /**
     * Derivative gain for damping.
     * Reacts to rate of change. Usually not needed for velocity control.
     */
    public static double VELOCITY_kD = 0.0;

    /**
     * Maximum integral accumulation to prevent windup.
     */
    public static double INTEGRAL_MAX = 0.3;

    /**
     * Maximum acceleration demand (ticks/sec^2).
     * Physical limit based on motor torque and flywheel inertia.
     */
    public static double MAX_ACCELERATION = 15000.0;

    /**
     * Velocity tolerance for "at target" check (ticks/sec).
     * Shooter is ready when within this tolerance.
     */
    public static double VELOCITY_TOLERANCE = 50.0;
}
