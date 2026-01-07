package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotConstants;

/**
 * Red Alliance TeleOp
 *
 * To enable PID tuning, change to: extends TeleOpTemplateTuning
 * To use clean version, keep: extends TeleOpTemplate
 */
@TeleOp
public class RedTeleOp extends TeleOpTemplate {
    private final utility.RobotHardware robot = utility.RobotHardware.getInstance();

    @Override
    public void initialize() {
        // Set alliance color BEFORE initHardware so it can use the correct fallback position
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Red;
        initHardware(false);

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (RobotConstants.UpdatableConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(RobotConstants.Pinpoint.redStartPoint);
            robot.pinpoint.update();
        }

        configureButtonBindings();
    }
}