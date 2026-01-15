package teleOps;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class RedTeleOp extends TeleOpTemplate {
    @Override
    public void initialize() {
        initForAlliance(EnumConstants.AllianceColor.Red, OdometryConstants.redStartPoint);
    }
}
