package Constants;

import com.bylazar.configurables.annotations.Configurable;


@Configurable
public class ShooterConstants {
    public static double DEFAULT_VELOCITY = 2200.0;
    public static double DEFAULT_DISTANCE = 100.0;
    public static double LUT_MAX_DISTANCE = 300.0;
    public static double HOOD_MIN_ANGLE = 30.0;
    public static double HOOD_MAX_ANGLE = 63.0;
    public static double HOOD_DEFAULT_ANGLE = 45.0;
    public static double HOOD_SERVO_AT_MIN_ANGLE = .95;
    public static double HOOD_SERVO_AT_MAX_ANGLE = 0.30;

    public static boolean SHOOTING_WHILE_MOVING_ENABLED = false;
    public static double SHOOTING_WHILE_MOVING_ACCEL_DEADBAND = 5.0;
    public static double SHOOTING_WHILE_MOVING_ANGULAR_ACCEL_DEADBAND = 0.1;
    public static double SHOOTING_WHILE_MOVING_ACCEL_FILTER_ALPHA = 0.3;
    public static double SHOOTING_WHILE_MOVING_MAX_VELOCITY_JUMP = 30.0;
    public static double SHOOTING_WHILE_MOVING_MAX_ACCEL = 120.0;
    public static double SHOOTING_WHILE_MOVING_MAX_ANGULAR_ACCEL = 8.0;
    public static double LEAD_VELOCITY_DEADBAND = 3.0;
    public static double LEAD_HEADING_VELOCITY_DEADBAND = 0.05;

    public static double VELOCITY_ADJUST_HARDCODED = 0.0;
    public static boolean MANUAL_OVERRIDE = false;
    public static double MANUAL_OVERRIDE_VELOCITY = 2000.0;
    public static double MANUAL_OVERRIDE_HOOD = 53.0;

    public static double TIME_IN_AIR = 0.675;

    public static double[][] VELOCITY_DATA = {
        {0, 1400},
        {43, 1400},
        {54, 1550},
        {64, 1700},
        {74, 1800},
        {84, 1900},
        {94, 2000},
        {105, 2100},
        {129, 2270},
        {136, 2350},
        {141, 2375},
        {146, 2440},
        {153, 2420},
        {158, 2440},
        {300, 2440}
    };

    public static double[][] HOOD_DATA = {
        {0, 30},
        {43, 30},
        {54, 35},
        {64, 41},
        {74, 45},
        {84, 48},
        {94, 51},
        {105, 53},
        {129, 55},
        {136, 55},
        {141, 57},
        {146, 57},
        {153, 54},
        {158, 55},
        {300, 55}
    };

    @Configurable
    public static class ShooterTuning {
        public static boolean TUNING_MODE = false;
        public static double TUNING_VELOCITY = 2000.0;
        public static double TUNING_HOOD_ANGLE = 53.0;

        public static boolean VOLTAGE_COMPENSATION_ENABLED = true;
        public static double NOMINAL_VOLTAGE = 13.0;


        public static double kS = 0.03;
        public static double kV = 3.2557046977E-4;
        public static double VELOCITY_kP = 0.001;
        public static double VELOCITY_TOLERANCE = 20.0;
        public static double RECOVERY_THRESHOLD = 15.0;
        public static double RECOVERY_VELOCITY_BOOST = 200.0;
        public static double SHOT_HOOD_COMPENSATION_Y_THRESHOLD = 52.0;
        public static double SHOT_HOOD_COMPENSATION_STEP_BACK = -0.6;
        public static double SHOT_HOOD_COMPENSATION_STEP_FRONT = 0.0;
    }
}
