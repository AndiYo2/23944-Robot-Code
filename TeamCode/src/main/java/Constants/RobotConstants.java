package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

public class RobotConstants {
    @Configurable
    public static class Robot {

        public static EnumConstants.AllianceColor allianceColor;

        public static final double MIN_SERVO_SAFE_POSITION = 0.0001;

        public static boolean ENABLE_TELEMETRY = true;
    }
    @Configurable
    public static class Controls {
        public static final double TRIGGER_THRESHOLD = 0.3;
        public static boolean SWAP_ALLIANCE_CONTROLS = true;
    }
}
