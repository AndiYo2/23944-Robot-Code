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

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();

    private ShootingSequenceManager sequenceManager;
    private ShootingValidator shootingValidator;

    private CatalogManager catalogManager;

    // PID Tuning Mode (for Turret)
    private boolean pidTuningMode = false;
    private int selectedPIDParameter = 0; // 0=P, 1=I, 2=D
    private double kP, kI, kD;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean bButtonPressed = false;

    // Step sizes for tuning - press B (in tuning mode) to cycle
    private double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 3; // Start with 0.0001

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();

        sequenceManager = new ShootingSequenceManager(spindexer, shooter);
        shootingValidator = new ShootingValidator(shooter, telemetry);
        catalogManager = new CatalogManager(spindexer, intake, telemetry, robot.intakeSensorPair);

        // Initialize PID tuning values from current turret settings
        kP = RobotConstants.Shooter.TURRET_PID.p;
        kI = RobotConstants.Shooter.TURRET_PID.i;
        kD = RobotConstants.Shooter.TURRET_PID.d;

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
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> catalogManager.initiateCataloging());

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> manualRotateCCW());

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> manualRotateCW());

        // PID Tuning Mode toggle
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)
                .whenPressed(() -> togglePIDTuningMode());

        // Mode toggles (when NOT in tuning mode)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    if (!pidTuningMode) {
                        shooter.toggleLimelightEnabled();
                    }
                });
    }

    @Override
    public void run() {
        super.run();

        updatePIDTuning();
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
    private void manualRotateCW() {
        if (sequenceManager.isExecuting()) {
            return; // Block manual rotation during shooting sequence
        }
        spindexer.rotateCW();
    }

    /**
     * Manual rotation backward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     */
    private void manualRotateCCW() {
        if (sequenceManager.isExecuting()) {
            return; // Block manual rotation during shooting sequence
        }
        spindexer.rotateCCW();
    }

    /**
     * Toggle PID tuning mode on/off
     */
    private void togglePIDTuningMode() {
        pidTuningMode = !pidTuningMode;
        if (pidTuningMode) {
            gamepad1.rumble(100); // Single short rumble on enable
        } else {
            gamepad1.rumble(500); // Long rumble on disable
        }
    }

    /**
     * Handle PID tuning controls when tuning mode is active
     */
    private void updatePIDTuning() {
        if (!pidTuningMode) return;

        // B button: Cycle step size
        if (gamepad1.b && !bButtonPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            gamepad1.rumble(50);
        }
        bButtonPressed = gamepad1.b;

        // D-pad left/right: Select parameter (P, I, or D)
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            selectedPIDParameter = (selectedPIDParameter - 1 + 3) % 3;
            gamepad1.rumble(50);
        }
        dpadLeftPressed = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !dpadRightPressed) {
            selectedPIDParameter = (selectedPIDParameter + 1) % 3;
            gamepad1.rumble(50);
        }
        dpadRightPressed = gamepad1.dpad_right;

        // D-pad up/down: Adjust selected parameter
        double increment = stepSizes[stepIndex]; // Use current step size

        if (gamepad1.dpad_up && !dpadUpPressed) {
            switch (selectedPIDParameter) {
                case 0: kP += increment; break;
                case 1: kI += increment; break;
                case 2: kD += increment; break;
            }
            updatePIDController();
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            switch (selectedPIDParameter) {
                case 0: kP = Math.max(0, kP - increment); break;
                case 1: kI = Math.max(0, kI - increment); break;
                case 2: kD = Math.max(0, kD - increment); break;
            }
            updatePIDController();
        }
        dpadDownPressed = gamepad1.dpad_down;
    }

    /**
     * Apply current PID values to the turret controller
     */
    private void updatePIDController() {
        shooter.setTurretPID(kP, kI, kD);
    }

    private void updateSubsystems() {
        spindexer.periodic();
        shooter.periodic();
        sequenceManager.update();
        catalogManager.update();



    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;

        // Refresh sensor readings for telemetry

        // PID TUNING MODE - Display prominently at top
        if (pidTuningMode) {
            telemetry.addLine("========================================");
            telemetry.addLine("🔧 TURRET PID TUNING MODE ACTIVE 🔧");
            telemetry.addLine("========================================");

            String[] paramNames = {"kP", "kI", "kD"};
            double[] paramValues = {kP, kI, kD};

            for (int i = 0; i < 3; i++) {
                String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                telemetry.addData(prefix + paramNames[i] + suffix,
                    String.format("%.6f", paramValues[i]));
            }

            telemetry.addLine("----------------------------------------");
            telemetry.addData("Step Size", "%.6f (Press B to change)", stepSizes[stepIndex]);
            telemetry.addLine("D-Pad Left/Right: Select parameter");
            telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.6f)", stepSizes[stepIndex]));
            telemetry.addLine("Back Button: Exit tuning mode");
            telemetry.addLine("========================================");
            telemetry.addData("Current PID", "new PIDCoefficients(%.5f, %.5f, %.5f)", kP, kI, kD);
            telemetry.addLine("========================================");
            telemetry.addLine("");
        }

        telemetry.addLine("Catalogging Debug stuff");
        telemetry.addData("Spindexer Pattern:", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
        telemetry.addData("Catalog State:", this.catalogManager.getState());
        telemetry.addLine("=== BALL DETECTION ===");
        DualBallDetector.Result result = robot.intakeSensorPair.detectBall();
        telemetry.addData("Ball Present", result.ballPresent);
        telemetry.addData("Ball Color", result.color);
        telemetry.addData("Confidence", "%.1f%%", result.confidence * 100);
        // Debug telemetry - always display
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
        telemetry.addData("Spindexer Pattern:", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());

        // Spindexer position debugging
        telemetry.addData("Spindexer Current:", String.format("%.1f°", spindexer.getServoPosition()));
        telemetry.addData("Spindexer Target:", String.format("%d°", spindexer.getTargetPosition()));
        telemetry.addData("Spindexer Slot Index:", spindexer.getSpindPosTracker());
        telemetry.addData("Rotation State:", spindexer.getRotationState());
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
            telemetry.addData("Target Pos:", String.format("%d°", spindexer.getTargetPosition()));
            telemetry.addData("Done Rotating?:", spindexer.isDoneRotating());
            telemetry.addData("Ready to Flip?:", spindexer.isReadyToFlip());
        }

        telemetry.update();
    }
}