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
import subsystems.Limelight;
import utility.*;

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();
    protected Limelight limelight;

    private ShootingSequenceManager sequenceManager;
    private CatalogManager catalogManager;
    private ShootingValidator shootingValidator;
    private SpindexerJamClearance jamClearance;

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();

        sequenceManager = new ShootingSequenceManager(spindexer, shooter, robot);
        catalogManager = new CatalogManager(robot.intakeSensor, spindexer, intake);
        shootingValidator = new ShootingValidator(shooter, telemetry);
        jamClearance = new SpindexerJamClearance(intake, spindexer);

        register(intake, shooter, spindexer);
    }

    protected void configureButtonBindings() {
        // Intake controls
        new Trigger(() -> gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> {
                    // Don't stop intake if CatalogManager is controlling it
                    if (!catalogManager.isCataloging()) {
                        intake.stopIntake();
                    }
                });

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
                .whenPressed(() -> manualRotateForward());

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> manualRotateBackward());

        // Mode toggles
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> toggleShootingStrategy());
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
        spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
        rotateBallPatternForward();
    }

    /**
     * Manual rotation backward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     */
    private void manualRotateBackward() {
        if (sequenceManager.isExecuting()) {
            return; // Block manual rotation during shooting sequence
        }
        spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_BACKWARD);
        rotateBallPatternBackward();
    }

    /**
     * Toggle between SMART and FAST shooting strategy modes
     */
    private void toggleShootingStrategy() {
        if (ShootingStrategy.getStrategyMode() == ShootingStrategy.StrategyMode.SMART) {
            ShootingStrategy.setStrategyMode(ShootingStrategy.StrategyMode.FAST);
        } else {
            ShootingStrategy.setStrategyMode(ShootingStrategy.StrategyMode.SMART);
        }
    }

    /**
     * Rotates the ball pattern forward to match physical spindexer rotation.
     * When spindexer rotates forward 120°:
     * - What was in slot 2 (storage) is now in slot 0 (intake)
     * - What was in slot 0 (intake) is now in slot 1 (shooter)
     * - What was in slot 1 (shooter) is now in slot 2 (storage)
     */
    private void rotateBallPatternForward() {
        RobotConstants.Enums.BallColor slot0 = robot.spindexerPattern.getBallInSlotX(0);
        RobotConstants.Enums.BallColor slot1 = robot.spindexerPattern.getBallInSlotX(1);
        RobotConstants.Enums.BallColor slot2 = robot.spindexerPattern.getBallInSlotX(2);

        robot.spindexerPattern.setBallPattern(slot2, slot0, slot1);
    }

    /**
     * Rotates the ball pattern backward to match physical spindexer rotation.
     * When spindexer rotates backward 120°:
     * - What was in slot 1 (shooter) is now in slot 0 (intake)
     * - What was in slot 2 (storage) is now in slot 1 (shooter)
     * - What was in slot 0 (intake) is now in slot 2 (storage)
     */
    private void rotateBallPatternBackward() {
        RobotConstants.Enums.BallColor slot0 = robot.spindexerPattern.getBallInSlotX(0);
        RobotConstants.Enums.BallColor slot1 = robot.spindexerPattern.getBallInSlotX(1);
        RobotConstants.Enums.BallColor slot2 = robot.spindexerPattern.getBallInSlotX(2);

        robot.spindexerPattern.setBallPattern(slot1, slot2, slot0);
    }

    private void updateSubsystems() {
        spindexer.periodic();
        shooter.periodic();
        sequenceManager.update();
        jamClearance.periodic();

        // Update catalog manager (tracks intake releases to catalog balls)
        // Disable cataloging during shooting sequence to prevent rotation conflicts
        boolean intakeActive = gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD;
        if (!sequenceManager.isExecuting()) {
            catalogManager.update(intakeActive);
        }

        // Runtime PID tuning (hold BACK + D-pad)
        if (gamepad1.back) {
            if (gamepad1.dpad_up) {
                spindexer.adjustP(0.0001);  // Increase P
            } else if (gamepad1.dpad_down) {
                spindexer.adjustP(-0.0001); // Decrease P
            }
            if (gamepad1.dpad_right) {
                spindexer.adjustD(0.00001);  // Increase D
            } else if (gamepad1.dpad_left) {
                spindexer.adjustD(-0.00001); // Decrease D
            }
        }
    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;

        // Refresh sensor readings for telemetry
        robot.intakeSensor.refreshScan();

        // Show PID tuning values (when in tuning mode)
        if (gamepad1.back) {
            telemetry.addLine("=== PID TUNING MODE ===");
            telemetry.addData("P (D-Pad Up/Down)", "%.5f", spindexer.getKP());
            telemetry.addData("D (D-Pad Left/Right)", "%.5f", spindexer.getKD());
            telemetry.addLine("Hold BACK + use D-pad to adjust");
            telemetry.addLine("=======================");
        }

        telemetry.addData("Shooting Mode:", ShootingStrategy.getStrategyMode());
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
        telemetry.addData("Spindexer Pattern:", getSpindexerPatternString());
        telemetry.addData("Catalog Status:", catalogManager.getStatus());

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
        telemetry.addData("Jam Clearance:", jamClearance.getStatus());

        // Debug: Show position info when sequence is stuck
        if (sequenceManager.isExecuting()) {
            telemetry.addData("Current Pos:", String.format("%.1f°", spindexer.getServoPosition()));
            telemetry.addData("Target Pos:", String.format("%.1f°", spindexer.getTargetPosition()));
            telemetry.addData("Done Rotating?:", spindexer.isDoneRotating());
            telemetry.addData("Ready to Flip?:", spindexer.isReadyToFlip());
        }

        telemetry.update();
    }

    private String getSpindexerPatternString() {
        return String.format("[I:%s S:%s T:%s]",
                robot.spindexerPattern.getBallInSlotX(0),  // I = Intake
                robot.spindexerPattern.getBallInSlotX(1),  // S = Shooter
                robot.spindexerPattern.getBallInSlotX(2)); // T = sTorage
    }
}