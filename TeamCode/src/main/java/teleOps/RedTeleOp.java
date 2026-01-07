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
        initHardware(false);
        RobotConstants.UpdatableConstants.allianceColor = RobotConstants.Enums.AllianceColor.Red;

        // Red alliance starts at different position (87.5, 8.5, 90°)
        robot.pinpoint.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                87.5,
                8.5,
                org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES,
                90));
        robot.pinpoint.update();

        configureButtonBindings();
    }
}