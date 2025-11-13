package TeleOps;

import Subsystems.Intake;
import Subsystems.Shooter;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import OldSystems.IMPLEMENTED.MecanumDriveOLD;
import utility.RobotHardware;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

@TeleOp
public class MainTeleOp extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driver;
    private MecanumDriveOLD drivetrain;
    private Intake intake;
    private Shooter shooter;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        driver = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driver);

        drivetrain = new MecanumDriveOLD();
        intake = new Intake();
        shooter = new Shooter();

        // Would add telemetry here
    }

    private void displayTelemetry() {
        addTelemetry("Robot Heading", mecanumDrive.getRobotHeading());
        addTelemetry("Shooter Power", outtakeMotors.getFlywheelSpeed());

    }

    private void addTelemetry(String label, String value) {
        telemetry.addData(label, value);
        telemetry.update();
    }

    private void addTelemetry(String label, double value) {
        telemetry.addData(label, value);
        telemetry.update();
    }
}