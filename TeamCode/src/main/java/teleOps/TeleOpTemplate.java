package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.MecanumDrive;
import subsystems.Shooter;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.*;
import utility.Shooting.ShootingSequenceManager;
import utility.Shooting.ShootingValidator;

import static utility.RobotConstants.Spindexer.spindexerPattern;

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();

    private ShootingSequenceManager sequenceManager;
    private ShootingValidator shootingValidator;

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();

        sequenceManager = new ShootingSequenceManager(spindexer, shooter);
        shootingValidator = new ShootingValidator(shooter, telemetry);

        register(intake, shooter, spindexer);
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
                .whenPressed(() -> manualRotateBackward());

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> manualRotateForward());

        // Mode toggles
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

    /**
     * Manual rotation forward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     */
    private void manualRotateForward() {
        if (sequenceManager.isExecuting()) {
            return; // Block manual rotation during shooting sequence
        }
        spindexer.rotateCW();
    }

    /**
     * Manual rotation backward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     */
    private void manualRotateBackward() {
        if (sequenceManager.isExecuting()) {
            return; // Block manual rotation during shooting sequence
        }
        spindexer.rotateCCW();
    }

    /**
     * Toggle between SMART and FAST shooting strategy modes
     */



    private void updateSubsystems() {
        spindexer.periodic();
        shooter.periodic();
        sequenceManager.update();



    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;

        // Refresh sensor readings for telemetry
        robot.intakeSensor.refreshScan();


        telemetry.addData("Executing Sequence:", sequenceManager.isExecuting());
        if (sequenceManager.isExecuting()) {
            telemetry.addData("Sequence Status:", sequenceManager.getStatus());
        }
        telemetry.addData("Shooter Power:", robot.shooterMotor2.getVelocity());
        telemetry.addData("Shooter distance:", shooter.getDistanceToTarget());

        // Turret debug telemetry
        telemetry.addData("Turret Pos:", String.format("%.1f°", shooter.getTurretPosition()));
        telemetry.addData("Turret Target:", String.format("%.1f°", shooter.getTargetTurretAngle()));
        telemetry.addData("Turret Error:", String.format("%.1f°", shooter.getTargetTurretAngle() - shooter.getTurretPosition()));
        telemetry.addData("Intake Color:", robot.intakeSensor.getBallColor());
        telemetry.addData("Spindexer Pattern:", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());

        // Spindexer position debugging
        telemetry.addData("Spindexer Current:", String.format("%.1f°", spindexer.getServoPosition()));
        telemetry.addData("Spindexer Target:", String.format("%.1f°", spindexer.getTargetPosition()));
        telemetry.addData("Position Error:", String.format("%.1f°",
            spindexer.getTargetPosition() - spindexer.getServoPosition()));
        telemetry.addData("Field Zone:", shooter.getFieldState());
        telemetry.addData("Shooting Status:", shootingValidator.getStatus(overrideRequested));
        telemetry.addData("Drive State:", mecanumDrive.getCurrentState());
        telemetry.addData("Intake State:", intake.getCurrentState());
        telemetry.addData("Shooter State:", shooter.getCurrentState());
        telemetry.addData("Spindexer State:", spindexer.getCurrentState());

        // Debug: Show position info when sequence is stuck
        if (sequenceManager.isExecuting()) {
            telemetry.addData("Current Pos:", String.format("%.1f°", spindexer.getServoPosition()));
            telemetry.addData("Target Pos:", String.format("%.1f°", spindexer.getTargetPosition()));
            telemetry.addData("Done Rotating?:", spindexer.isDoneRotating());
            telemetry.addData("Ready to Flip?:", spindexer.isReadyToFlip());
        }

        telemetry.update();
    }
}