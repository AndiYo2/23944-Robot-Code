package tests;

import Constants.DriveConstants;
import Constants.EnumConstants;
import Constants.RobotConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.Intake;
import subsystems.MecanumDrive;
import utility.RobotHardware;

/**
 * Drive + Intake only TeleOp.
 * No shooter, turret, spindexer, limelight, or odometry.
 */
@TeleOp
public class IntakeOnlyTeleOp extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driverGamepad;

    private MecanumDrive mecanumDrive;
    private Intake intake;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private final double[] controlsArray = new double[3];

    @Override
    public void initialize() {
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Red;

        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();

        register(mecanumDrive, intake);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Drive with left stick + right stick rotation
        mecanumDrive.setDefaultCommand(
                new RunCommand(() -> {
                    double[] controls = getTransformedControls();
                    mecanumDrive.setDynamicSpeedMultiplier(getDynamicSlowMultiplier());
                    mecanumDrive.drive(controls[0], controls[1], controls[2]);
                }, mecanumDrive)
        );

        // Right bumper — intake (hold to run)
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> intake.runIntake()))
                .whenReleased(new InstantCommand(() -> intake.stopIntake()));

        // Dpad Right — reverse intake (hold)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(intake::reverse))
                .whenReleased(new InstantCommand(intake::stopIntake));

        // B — toggle slow mode
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(mecanumDrive::toggleSlowMode));

        // Start — reset IMU/yaw
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(mecanumDrive::resetYaw));
    }

    @Override
    public void run() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        robot.clearBulkCache();

        super.run();

        telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
        telemetry.update();
    }

    private double[] getTransformedControls() {
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRotation = gamepad1.right_stick_x;

        EnumConstants.AllianceColor invertedAlliance = RobotConstants.Controls.SWAP_ALLIANCE_CONTROLS
                ? EnumConstants.AllianceColor.Blue
                : EnumConstants.AllianceColor.Red;

        if (RobotConstants.Robot.allianceColor != null &&
                RobotConstants.Robot.allianceColor == invertedAlliance) {
            controlsArray[0] = -rawY;
            controlsArray[1] = -rawX;
            controlsArray[2] = rawRotation;
        } else {
            controlsArray[0] = rawY;
            controlsArray[1] = rawX;
            controlsArray[2] = rawRotation;
        }
        return controlsArray;
    }

    private double getDynamicSlowMultiplier() {
        double trigger = gamepad1.left_trigger;
        if (trigger < DriveConstants.DYNAMIC_SLOW_DEADBAND) {
            return 1.0;
        }
        double normalized = (trigger - DriveConstants.DYNAMIC_SLOW_DEADBAND)
                / (1.0 - DriveConstants.DYNAMIC_SLOW_DEADBAND);
        return 1.0 - normalized * (1.0 - DriveConstants.DYNAMIC_SLOW_MIN);
    }
}