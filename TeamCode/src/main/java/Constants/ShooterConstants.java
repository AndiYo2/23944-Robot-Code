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

    /** Maximum distance covered by the velocity/hood LUTs (inches). Values beyond this are clamped. */
    public static double LUT_MAX_DISTANCE = 300.0;

    // Hood angle: 0 = vertical, 90 = horizontal. Usable range: 30-63 degrees
    public static double HOOD_MIN_ANGLE = 30.0;
    public static double HOOD_MAX_ANGLE = 63.0;
    public static double HOOD_DEFAULT_ANGLE = 45.0;
    public static double HOOD_SERVO_AT_MIN_ANGLE = .95;
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.30;

    /** Enable Shooting_While_Moving for shoot-while-moving compensation.
     *  Uses full kinematic prediction: pos + vel*t + 0.5*accel*t²
     *  Computes turret angle, flywheel velocity, and hood angle from predicted future pose. */
    public static boolean SHOOTING_WHILE_MOVING_ENABLED = false;

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

    /** Competition-time manual override. When true: turret centers, velocity/hood use fixed values below. */
    public static boolean MANUAL_OVERRIDE = false;
    public static double MANUAL_OVERRIDE_VELOCITY = 2000.0;
    public static double MANUAL_OVERRIDE_HOOD = 53.0;

    // Time in air (seconds)
    public static double TIME_IN_AIR = 0.675;

    // Velocity LUT - Distance (inches) -> Velocity (ticks/sec)
    public static double[][] VELOCITY_DATA = {
        {0, 1400},
        {42, 1400},
        {53, 1550},
        {63, 1700},
        {73, 1800},
        {83, 1900},
        {93, 2000},
        {104, 2100},
        {128, 2270},
        {135, 2350},
        {140, 2375},
        {145, 2440},
        {152, 2420},
        {157, 2440},
        {300, 2440}
    };

    // Hood Angle LUT - Distance (inches) -> Hood Angle (degrees, 0=vertical, 90=horizontal)
    public static double[][] HOOD_DATA = {
        {0, 30},
        {42, 30},
        {53, 35},
        {63, 41},
        {73, 45},
        {83, 48},
        {93, 51},
        {104, 53},
        {128, 55},
        {135, 55},
        {140, 57},
        {145, 57},
        {152, 54},
        {157, 55},
        {300, 55}
    };

    /**
     * All shooter controller tuning: feedforward, bang-bang recovery, hood compensation,
     * voltage compensation, and manual override values.
     * Use Panels to adjust values in real-time.
     */
    @Configurable
    public static class ShooterTuning {
        /** Enable to override automatic velocity/hood angle with manual values */
        public static boolean TUNING_MODE = false;

        /** Manual velocity setting (ticks/sec) - only used when TUNING_MODE is true */
        public static double TUNING_VELOCITY = 2000.0;

        /** Manual hood angle setting (degrees) - only used when TUNING_MODE is true */
        public static double TUNING_HOOD_ANGLE = 53.0;

        // ==================== VOLTAGE COMPENSATION ====================

        /** Enable battery voltage compensation to maintain consistent flywheel speed as voltage sags */
        public static boolean VOLTAGE_COMPENSATION_ENABLED = true;
        /** Nominal battery voltage used as the reference for compensation (volts) */
        public static double NOMINAL_VOLTAGE = 13.0;

        // ==================== FEEDFORWARD ====================

        /** Static friction compensation */
        public static double kS = 0.03;

        /** Velocity gain */
        public static double kV = 3.2557046977E-4;

        // === LEGACY PID CONSTANTS (replaced by bang-bang + FF+P) ===
        // public static double kA = 0.0005;
        // public static double VELOCITY_kI = 0;
        // public static double VELOCITY_kD = 0.0;
        // public static double INTEGRAL_MAX = 0.3;
        // public static double MAX_ACCELERATION = 15000.0;
        // === END LEGACY ===

        // ==================== BANG-BANG RECOVERY ====================

        /** Proportional gain — used in MAINTAIN mode for steady-state accuracy */
        public static double VELOCITY_kP = 0.001;

        /** Velocity tolerance for isAtTargetVelocity() */
        public static double VELOCITY_TOLERANCE = 20.0;

        /** Error threshold (ticks/sec) above which controller uses RECOVERY mode (full power).
         *  Below this, uses MAINTAIN mode (FF + P). Must be > VELOCITY_TOLERANCE to avoid chatter. */
        public static double RECOVERY_THRESHOLD = 15.0;

        /** Extra velocity (ticks/sec) added to effective setpoint during RECOVERY mode.
         *  Keeps motor at full power slightly past the real setpoint; the flywheel's high MOI
         *  absorbs the overshoot. Start at 0, increase by 25 until recovery time is minimized
         *  without velocity ringing. */
        public static double RECOVERY_VELOCITY_BOOST = 200.0;

        // ==================== HOOD COMPENSATION ====================

        /** Robot Y threshold (inches) separating back-zone from front-zone shots for per-shot hood
         *  compensation. When cachedPoseY ≤ this value, SHOT_HOOD_COMPENSATION_STEP_BACK is applied
         *  per shot; otherwise SHOT_HOOD_COMPENSATION_STEP_FRONT. */
        public static double SHOT_HOOD_COMPENSATION_Y_THRESHOLD = 52.0;

        /** Hood compensation step applied per shot when robot Y is in the back zone (0–52").
         *  Empirically found — long-range shots show consistent flywheel velocity drop that this
         *  step compensates for. */
        public static double SHOT_HOOD_COMPENSATION_STEP_BACK = -0.6;

        /** Hood compensation step applied per shot when robot Y > threshold (front zone). Close-range
         *  shots have enough flywheel energy margin that no per-shot hood drop is needed. */
        public static double SHOT_HOOD_COMPENSATION_STEP_FRONT = 0.0;
    }
}
