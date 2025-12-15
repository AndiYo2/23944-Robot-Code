package utility;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.*;
import subsystems.*;
import utility.RobotConstants.Enums.ShooterCases;

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;
    protected ColorSensorSubsytem colorSensor;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();
    protected Limelight limelight;

    private ShootingSequenceManager sequenceManager;

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        colorSensor = new ColorSensorSubsytem();
        spindexer = new Spindexer();

        sequenceManager = new ShootingSequenceManager(spindexer, shooter, robot);

        register(intake, shooter, spindexer, colorSensor);
    }

    protected void configureButtonBindings() {
        // Intake controls
        new Trigger(() -> gamepad1.left_trigger > 0.3)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> intake.stopIntake());

        // Shooting controls
        new Trigger(() -> gamepad1.right_trigger > 0.3)
                .whenActive(() -> sequenceManager.startShootingSequence());

        // Drive controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(() -> mecanumDrive.resetYaw());
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> mecanumDrive.toggleSlowMode());
        // Manual shooting controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> shooter.shootBall());
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.flickBallOut());

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> spindexer.rotate(120));

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> spindexer.rotate(-120));

        // Mode toggles
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> sequenceManager.toggleShootingMode());
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> shooter.toggleLimelight());
    }

    @Override
    public void run() {
        super.run();

        updateDrivetrain();
        updateSubsystems();
        updateTelemetry();
    }

    private void updateDrivetrain() {
        mecanumDrive.drive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );
    }

    private void updateSubsystems() {
        spindexer.periodic();
        sequenceManager.update();
    }

    private void updateTelemetry() {
        telemetry.addData("Servo Pos:", spindexer.getServoPosition());
        telemetry.addData("GoalServoPos:", spindexer.getTargetPosition());
        telemetry.addData("Shooting Mode:", ShootingStrategy.getStrategyMode());
        telemetry.addData("Executing Sequence:", sequenceManager.isExecuting());
        telemetry.addData("Shooter Power:", shooter.getRequiredVelocity());
        telemetry.addData("Color Detected:", colorSensor.getBallColor());
        telemetry.addData("Spindexer Pattern:", getSpindexerPatternString());
        telemetry.update();
    }

    private String getSpindexerPatternString() {
        return robot.spindexerPattern.getBallInSlotX(0) + ", " +
                robot.spindexerPattern.getBallInSlotX(1) + ", " +
                robot.spindexerPattern.getBallInSlotX(2);
    }
}