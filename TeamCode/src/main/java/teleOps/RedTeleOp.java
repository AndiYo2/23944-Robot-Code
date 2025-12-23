package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotConstants;
import utility.RobotHardware;
import utility.TeleOpTemplate;


@TeleOp
public class RedTeleOp extends TeleOpTemplate {

    @Override
    public void initialize() {
        initHardware(false);
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Red;
        configureButtonBindings();
    }
}