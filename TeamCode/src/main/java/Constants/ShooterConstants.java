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
    public static double HOOD_SERVO_AT_MIN_ANGLE = 1;
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.38;

    public static boolean SHOOT_WHILE_MOVING_ENABLED = false;

    /** Minimum robot speed (in/sec) for shoot-while-moving lead compensation to activate in TeleOp */
    public static double MOVING_WHILE_SHOOTING_VELOCITY_THRESHOLD = 2.0;

    // Lead compensation filtering
    /** Minimum translational speed (in/sec) for lead compensation. Below this, robot is considered stationary. */
    public static double LEAD_VELOCITY_DEADBAND = 3.0;
    /** Minimum heading velocity (rad/sec) for heading lead compensation. Below this, heading is considered stable. */
    public static double LEAD_HEADING_VELOCITY_DEADBAND = 0.05;

    // Time in air (seconds)
    public static double TIME_IN_AIR = 0.675;

    // Velocity LUT - Distance (inches) -> Velocity (ticks/sec)
    public static double[][] VELOCITY_DATA = {
        {0, 1700},
        {38, 1700},
        {49, 1740},
        {58, 1780},
        {68.5, 1900},
        {76.5, 2020},
        {85, 2060},
        {101, 2280},
        {128, 2550},
        {140, 2620},
        {148, 2680},
        {151, 2700},
        {160, 2880},
        {300, 2880}
    };

    // Hood Angle LUT - Distance (inches) -> Hood Angle (degrees, 0=vertical, 90=horizontal)
    public static double[][] HOOD_DATA = {
        {0, 32},
        {38, 32},
        {49, 35},
        {58, 37},
        {68.5, 42},
        {76.5, 45},
        {85, 46.5},
        {101, 48},
        {128, 52},
        {140, 53},
        {148, 54},
        {151, 54.5},
        {160, 54.5},
        {300, 54.5}
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

    // Voltage compensation
    /** Enable battery voltage compensation to maintain consistent flywheel speed as voltage sags */
    public static boolean VOLTAGE_COMPENSATION_ENABLED = false;
    /** Nominal battery voltage used as the reference for compensation (volts) */
    public static double NOMINAL_VOLTAGE = 13.0;

    // Feedforward + PID velocity control

    /** Static friction compensation */
    public static double kS = 0.03;

    /** Velocity gain */
    public static double kV = 0.00032;

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
    public static double VELOCITY_TOLERANCE = 50.0;
}
