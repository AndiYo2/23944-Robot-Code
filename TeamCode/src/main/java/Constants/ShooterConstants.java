package Constants;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Shooter flywheel and velocity constants.
 */
@Configurable
public class ShooterConstants {
    // Velocity defaults
    public static double DEFAULT_VELOCITY = 2200.0;
    public static double DEFAULT_DISTANCE = 100.0;

    // Hood angle: 0 = vertical, 90 = horizontal. Usable range: 30-63 degrees
    public static double HOOD_MIN_ANGLE = 30.0;
    public static double HOOD_MAX_ANGLE = 63.0;
    public static double HOOD_DEFAULT_ANGLE = 45.0;
    public static double HOOD_SERVO_AT_MIN_ANGLE = .95;
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.30;

    /** Enable Shooting_While_Moving for shoot-while-moving compensation.
     *  Uses full kinematic prediction: pos + vel*t + 0.5*accel*t²
     *  Computes turret angle, flywheel velocity, and hood angle from predicted future pose. */
    public static boolean SHOOTING_WHILE_MOVING_ENABLED = true;

    /** Minimum translational acceleration magnitude (in/s²) to include in prediction.
     *  Below this threshold, acceleration is zeroed to prevent noise amplification. */
    public static double SHOOTING_WHILE_MOVING_ACCEL_DEADBAND = 5.0;

    /** Minimum angular acceleration (rad/s²) to include in heading prediction.
     *  Below this threshold, angular acceleration is zeroed. */
    public static double SHOOTING_WHILE_MOVING_ANGULAR_ACCEL_DEADBAND = 0.1;

    /** Low-pass EMA filter alpha for acceleration smoothing (0-1).
     *  Higher = more responsive but noisier. Lower = smoother but more latent.
     *  0.3 is a good starting point. */
    public static double SHOOTING_WHILE_MOVING_ACCEL_FILTER_ALPHA = 0.3;

    /** Maximum velocity change per loop (in/s) before it's considered a collision.
     *  Anything above this in a single loop is physically impossible from motors alone.
     *  At 50Hz, 30 in/s per loop = 1500 in/s² — well above any motor-driven acceleration. */
    public static double SHOOTING_WHILE_MOVING_MAX_VELOCITY_JUMP = 30.0;

    /** Maximum allowed acceleration magnitude (in/s²) for prediction.
     *  The robot physically cannot accelerate faster than ~120 in/s² under its own power.
     *  Anything above this is external (collision, getting pushed). */
    public static double SHOOTING_WHILE_MOVING_MAX_ACCEL = 120.0;

    /** Maximum allowed angular acceleration (rad/s²) for prediction. */
    public static double SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL = 8.0;

    /** Minimum translational speed (in/sec) for lead compensation. Below this, robot is considered stationary. */
    public static double LEAD_VELOCITY_DEADBAND = 3.0;
    /** Minimum heading velocity (rad/sec) for heading lead compensation. Below this, heading is considered stable. */
    public static double LEAD_HEADING_VELOCITY_DEADBAND = 0.05;

    /** Flat velocity offset added to every LUT lookup (ticks/sec). Tune via Panels to compensate for SWM distance mismatch. */
    public static double VELOCITY_ADJUST_HARDCODED = 0.0;

    // Time in air (seconds)
    public static double TIME_IN_AIR = 0.675;

    // Velocity LUT - Distance (inches) -> Velocity (ticks/sec)
    public static double[][] VELOCITY_DATA = {
        {0, 1550},
        {40, 1550},
        {50, 1550},
        {60, 1650},
        {76, 1750},
        {90, 1870},
        {100, 1980},
        {110, 2020},
        {123, 2100},
        {130, 2350},
        {140.5, 2350},
        {147, 2370},
        {153, 2400},
        {160, 2450},
        {300, 2450}
    };

    // Hood Angle LUT - Distance (inches) -> Hood Angle (degrees, 0=vertical, 90=horizontal)
    public static double[][] HOOD_DATA = {
        {0, 32},
        {40, 32},
        {50, 35},
        {60, 40},
        {76, 45},
        {90, 48},
        {100, 49},
        {110, 49},
        {123, 50},
        {130, 56},
        {140.5, 56},
        {147, 56},
        {153, 56},
        {160, 56},
        {300, 56}
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
        public static double TUNING_VELOCITY = 2000.0;

        /** Manual hood angle setting (degrees) - only used when TUNING_MODE is true */
        public static double TUNING_HOOD_ANGLE = 53.0;
    }

    /** Enable battery voltage compensation to maintain consistent flywheel speed as voltage sags */
    public static boolean VOLTAGE_COMPENSATION_ENABLED = false;
    /** Nominal battery voltage used as the reference for compensation (volts) */
    public static double NOMINAL_VOLTAGE = 13.0;

    // Feedforward + PID velocity control

    /** Static friction compensation */
    public static double kS = 0.03;

    /** Velocity gain */
    public static double kV = 0.00029;

    /** Acceleration gain */
    public static double kA = 0.00001;

    /** Proportional gain */
    public static double VELOCITY_kP = 0.002;

    /** Integral gain */
    public static double VELOCITY_kI = 0.00001;

    /** Derivative gain */
    public static double VELOCITY_kD = 0.0;

    /** Maximum integral accumulation */
    public static double INTEGRAL_MAX = 0.3;
    public static double MAX_ACCELERATION = 15000.0;

    /** Velocity tolerance */
    public static double VELOCITY_TOLERANCE = 20.0;
}
