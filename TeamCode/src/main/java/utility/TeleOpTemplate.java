package utility;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.MecanumDrive;
import subsystems.Shooter;
import subsystems.Intake;
import subsystems.Spindexer;
import subsystems.ColorSensorSubsytem;
import subsystems.Limelight;

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
    private CatalogManager catalogManager;
    private ShootingValidator shootingValidator;

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        colorSensor = new ColorSensorSubsytem();
        spindexer = new Spindexer();

        sequenceManager = new ShootingSequenceManager(spindexer, shooter, robot);
        catalogManager = new CatalogManager(colorSensor, spindexer, intake);
        shootingValidator = new ShootingValidator(shooter, telemetry);

        register(intake, shooter, spindexer, colorSensor);
    }

    protected void configureButtonBindings() {
        // Intake controls
        new Trigger(() -> gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> intake.stopIntake());

        // Shooting controls (with zone validation)
        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> attemptShoot());

        // Drive controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(() -> mecanumDrive.resetYaw());
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(() -> mecanumDrive.toggleSlowMode());
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.triggerFlick());
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> attemptShootingSequence());

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD));

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_BACKWARD));

        // Mode toggles
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> sequenceManager.toggleShootingMode());
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> shooter.toggleLimelightEnabled());
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

    /**
     * Attempts to shoot with zone validation
     * Checks if robot is in shooting zone or override is active
     */
    private void attemptShoot() {
        // Override: Click right joystick (right_stick_button) to shoot outside zone
        boolean overrideRequested = gamepad1.right_stick_button;

        if (shootingValidator.canShoot(overrideRequested)) {
            shooter.triggerShot();
        } else {
            // Shooting blocked - provide haptic feedback
            gamepad1.rumble(200);
        }
    }

    /**
     * Attempts to start shooting sequence with zone validation
     */
    private void attemptShootingSequence() {
        // Override: Click right joystick (right_stick_button) to shoot outside zone
        boolean overrideRequested = gamepad1.right_stick_button;

        if (shootingValidator.canShoot(overrideRequested)) {
            sequenceManager.startShootingSequence();
        } else {
            // Shooting blocked - provide haptic feedback
            gamepad1.rumble(200);
        }
    }

    private void updateSubsystems() {
        spindexer.periodic();
        sequenceManager.update();

        // Update catalog manager (tracks intake releases to catalog balls)
        boolean intakeActive = gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD;
        catalogManager.update(intakeActive);
    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;


        telemetry.addData("Shooting Mode:", ShootingStrategy.getStrategyMode());
        telemetry.addData("Executing Sequence:", sequenceManager.isExecuting());
        telemetry.addData("Shooter Power:", shooter.getRequiredVelocity());
        telemetry.addData("Color Detected:", colorSensor.getBallColor());
        telemetry.addData("Spindexer Pattern:", getSpindexerPatternString());
        telemetry.addData("Catalog Status:", catalogManager.getStatus());
        telemetry.addData("Field Zone:", shooter.getFieldState());
        telemetry.addData("Shooting Status:", shootingValidator.getStatus(overrideRequested));
        telemetry.addData("Drive State:", mecanumDrive.getCurrentState());
        telemetry.addData("Intake State:", intake.getCurrentState());
        telemetry.addData("Color State:", colorSensor.getCurrentState());
        telemetry.addData("Shooter State:", shooter.getCurrentState());
        telemetry.addData("Spindexer State:", spindexer.getCurrentState());
        telemetry.update();
    }

    private String getSpindexerPatternString() {
        return robot.spindexerPattern.getBallInSlotX(0) + ", " +
                robot.spindexerPattern.getBallInSlotX(1) + ", " +
                robot.spindexerPattern.getBallInSlotX(2);
    }
}