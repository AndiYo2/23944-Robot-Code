package Constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

/**
 * Goal position definitions.
 */
@Configurable
public class FieldMap {

    private static Pose cachedGoalPosition = null;
    private static EnumConstants.AllianceColor cachedAllianceColor = null;

    public static Pose getGoalPosition() {
        EnumConstants.AllianceColor current = RobotConstants.Robot.allianceColor;
        if (cachedGoalPosition == null || current != cachedAllianceColor) {
            cachedAllianceColor = current;
            if (current == EnumConstants.AllianceColor.Red) {
                cachedGoalPosition = new Pose(OdometryConstants.RED_GOAL_X, OdometryConstants.RED_GOAL_Y, 0);
            } else {
                cachedGoalPosition = new Pose(OdometryConstants.BLUE_GOAL_X, OdometryConstants.BLUE_GOAL_Y, 0);
            }
        }
        return cachedGoalPosition;
    }
}
