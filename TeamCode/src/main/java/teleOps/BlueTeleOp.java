package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotConstants;


@TeleOp
public class BlueTeleOp extends TeleOpTemplate {

    @Override
    public void initialize() {
        initHardware(false);
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Blue;
        configureButtonBindings();
    }
}