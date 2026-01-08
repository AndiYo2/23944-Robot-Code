package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotConstants;

/**
 * Blue Alliance TeleOp
 *
 * To enable PID tuning, change to: extends TeleOpTemplateTuning
 * To use clean version, keep: extends TeleOpTemplate
 */
@TeleOp
public class BlueTeleOp extends TeleOpTemplateTuning {
    private final utility.RobotHardware robot = utility.RobotHardware.getInstance();

    @Override
    public void initialize() {
        // Set alliance color BEFORE initHardware so it can use the correct fallback position
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Blue;
        initHardware(false);

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (RobotConstants.UpdatableConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(RobotConstants.Pinpoint.blueStartPoint);
            robot.pinpoint.update();
        }

        configureButtonBindings();
    }
}