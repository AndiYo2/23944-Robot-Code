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

        // Time to run intake per ball during catalogging
        public static double BALL_INTAKE_TIME = 0.5;

        // Time to wait for spindexer to reach position
        public static double POSITION_TIMEOUT_SECONDS = 1.0;
    }
    @Configurable
    public static class ShootingSequenceV2 {
        // Time to wait after spindexer flipper extends before considering it done
        public static double SPINDEXER_FLICK_TIME = 0.075;

        // Time to wait after spindexer flipper retracts before rotating
        public static double POST_FLICK_SETTLE_TIME = 0.02;

        // Time to wait after rotation before ready
        public static double POST_ROTATION_SETTLE_TIME = 0.05;

        // Time to wait after shooter flipper fires
        public static double POST_SHOT_SETTLE_TIME = 0.02;
    }

    public static class Controls {
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
}
