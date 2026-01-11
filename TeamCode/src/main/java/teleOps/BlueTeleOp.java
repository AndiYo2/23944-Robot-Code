package teleOps;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.RobotHardware;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class BlueTeleOp extends TeleOpTemplate {
    private final RobotHardware robot = RobotHardware.getInstance();

    @Override
    public void initialize() {
        // Set alliance color BEFORE initHardware so it can use the correct fallback position
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Blue;
        initHardware(false);

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (OdometryConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(OdometryConstants.blueStartPoint);
            robot.pinpoint.update();
        }

        configureButtonBindings();
    }
}