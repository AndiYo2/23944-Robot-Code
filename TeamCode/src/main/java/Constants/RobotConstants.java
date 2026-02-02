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

        public static final double MIN_SERVO_SAFE_POSITION = 0.0001;

        // Toggle Panels telemetry on/off — disable for competition to save loop time
        public static boolean ENABLE_TELEMETRY = true;
    }

    public static class Controls {
        public static final double TRIGGER_THRESHOLD = 0.3;

        // Toggle to swap Red/Blue control mapping for testing
        // When true: Red uses standard controls, Blue uses inverted controls
        // When false (default): Blue uses standard controls, Red uses inverted controls
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
}
