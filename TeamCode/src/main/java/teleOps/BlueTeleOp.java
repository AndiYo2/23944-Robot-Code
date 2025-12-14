package teleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotHardware;
import utility.TeleOpTemplate;


@TeleOp
public class BlueTeleOp extends TeleOpTemplate {

    @Override
    public void initialize() {
        initHardware(false);
        configureButtonBindings();
        RobotHardware.getInstance().limelight.pipelineSwitch(3);
    }
}