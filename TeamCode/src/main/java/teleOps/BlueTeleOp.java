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
public class BlueTeleOp extends TeleOpTemplate {

    @Override
    public void initialize() {
        initHardware(false);
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Blue;
        configureButtonBindings();
    }
}