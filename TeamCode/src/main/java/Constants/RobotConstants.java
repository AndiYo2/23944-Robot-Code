package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class RobotConstants {

    public static class Robot {
        // Robot dimensions (inches)
        public static final double ROBOT_SIZE = 17; // Robot width/length (square)
        public static final double HALF_SIZE = ROBOT_SIZE / 2.0; // Distance from center to edge

        public static EnumConstants.AllianceColor allianceColor;
    }

    @Configurable
    public static class Cataloging {
        // Timeout for scanning state (no ball detected)
        public static double SCAN_TIMEOUT_SECONDS = 2.0;

        // Timeout for rotation state (spindexer stuck)
        public static double ROTATION_TIMEOUT_SECONDS = 3.0;
    }

    public static class Encoder {
        public static double MAX_VOLTAGE = 3.3;
        public static double FULL_ROTATION_DEGREES = 360.0;

        public static double ANGLE_UPPER_BOUND = 180.0;
        public static double ANGLE_LOWER_BOUND = -180.0;
    }

    @Configurable
    public static class PID {
        // --- Delta Time Validation ---
        public static double DT_MAX = 1.0;                         // Max delta time (skip if exceeded)
        public static double DT_MIN = 0.001;                       // Min delta time (skip if below)
        public static double DT_DEFAULT = 0.02;                    // Default delta time (50Hz)

        // --- Anti-Windup ---
        public static double INTEGRAL_CLAMP_MAX = 50.0;            // Upper integral windup limit
        public static double INTEGRAL_CLAMP_MIN = -50.0;           // Lower integral windup limit
    }

    @Configurable
    public static class ShootingSequence {
        public static int TOTAL_BALLS = 3;
        public static double SHOOTER_FLIPPER_TIME = 0.02;
        public static double EXTRA_WAIT_TIME = 0.075;
        public static double SPINDEXER_SETTLING_TIME = 0;
    }

    public static class Controls {
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
}
