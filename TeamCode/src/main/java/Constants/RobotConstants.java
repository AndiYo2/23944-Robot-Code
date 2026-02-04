package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

public class RobotConstants {
    @Configurable
    public static class Robot {
        // Robot dimensions (inches) PLEASE PLEASE PLEASE UPDATE NEXT ITTERATION DO NOT FORGET SUPPER IMPORTANT AWOIDAWODUHIJASLKJ:DFA:LSKFJS
        public static final double ROBOT_SIZE = 17; // Robot width/length (square)
        public static final double HALF_SIZE = ROBOT_SIZE / 2.0; // Distance from center to edge

        public static EnumConstants.AllianceColor allianceColor;

        public static final double MIN_SERVO_SAFE_POSITION = 0.0001;

        public static boolean ENABLE_TELEMETRY = true;
    }
    @Configurable
    public static class Controls {
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
}
