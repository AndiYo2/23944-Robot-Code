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
    protected subsystems.Limelight limelight;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();

    private ShootingSequenceManager sequenceManager;
    private ShootingValidator shootingValidator;

    private CatalogManager catalogManager;

    // PIDF Tuning Mode
    private boolean pidTuningMode = false;
    private boolean tuningShooter = false; // false = spindexer, true = shooter
    private int selectedPIDParameter = 0; // 0=P, 1=I, 2=D, 3=F

    // Spindexer PIDF values
    private double spindexer_kP, spindexer_kI, spindexer_kD, spindexer_kF;

    // Shooter PIDF values
    private double shooter_kP, shooter_kI, shooter_kD, shooter_kF;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean bButtonPressed = false;
    private boolean yButtonPressed = false;

    // Step sizes for tuning - press B (in tuning mode) to cycle
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};
    private int stepIndex = 3; // Start with 0.01

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();
        limelight = new subsystems.Limelight();

        // Link Limelight subsystem to Shooter for dual-mode tracking
        shooter.setLimelightSubsystem(limelight);

        sequenceManager = new ShootingSequenceManager(spindexer, shooter);
        shootingValidator = new ShootingValidator(shooter, telemetry);
        catalogManager = new CatalogManager(spindexer, intake, telemetry, robot.intakeSensorPair);

        // Initialize PIDF tuning values from current settings
        spindexer_kP = RobotConstants.Spindexer.SPINDEXER_P;
        spindexer_kI = RobotConstants.Spindexer.SPINDEXER_I;
        spindexer_kD = RobotConstants.Spindexer.SPINDEXER_D;
        spindexer_kF = RobotConstants.Spindexer.SPINDEXER_F;

        shooter_kP = RobotConstants.Shooter.SHOOTER_P;
        shooter_kI = RobotConstants.Shooter.SHOOTER_I;
        shooter_kD = RobotConstants.Shooter.SHOOTER_D;
        shooter_kF = RobotConstants.Shooter.SHOOTER_F;

        register(intake, shooter, spindexer, limelight);
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
                        limelight.toggleMode();
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
        double[] controls = getTransformedControls();
        mecanumDrive.drive(
                controls[0],  // fieldY (forward/backward)
                controls[1],  // fieldX (strafe left/right)
                controls[2]   // rotation
        );
    }

    /**
     * Applies alliance-specific control mapping transformations.
     * Blue alliance uses standard field coordinates.
     * Red alliance inverts X and Y axes to account for mirrored starting position.
     *
     * @return double array [fieldY, fieldX, rotation]
     */
    private double[] getTransformedControls() {
        double rawY = -gamepad1.left_stick_y;  // Negative because gamepad Y is inverted
        double rawX = gamepad1.left_stick_x;
        double rawRotation = gamepad1.right_stick_x;

        // Check alliance color (defaults to Blue if not set)
        if (RobotConstants.UpdatableConstants.allianceColor != null &&
                RobotConstants.UpdatableConstants.allianceColor == RobotConstants.Enums.AllianceColor.Red) {
            // Red alliance: invert both translational axes (180° field rotation)
            return new double[] {-rawY, -rawX, rawRotation};
        } else {
            // Blue alliance: standard mapping
            return new double[] {rawY, rawX, rawRotation};
        }
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

        // Y button: Toggle between spindexer and shooter tuning
        if (gamepad1.y && !yButtonPressed) {
            tuningShooter = !tuningShooter;
            gamepad1.rumble(100); // Rumble to confirm switch
        }
        yButtonPressed = gamepad1.y;

        // B button: Cycle step size
        if (gamepad1.b && !bButtonPressed) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
            gamepad1.rumble(50);
        }
        bButtonPressed = gamepad1.b;

        // D-pad left/right: Select parameter (P, I, D, or F)
        if (gamepad1.dpad_left && !dpadLeftPressed) {
            selectedPIDParameter = (selectedPIDParameter - 1 + 4) % 4;
            gamepad1.rumble(50);
        }
        dpadLeftPressed = gamepad1.dpad_left;

        if (gamepad1.dpad_right && !dpadRightPressed) {
            selectedPIDParameter = (selectedPIDParameter + 1) % 4;
            gamepad1.rumble(50);
        }
        dpadRightPressed = gamepad1.dpad_right;

        // D-pad up/down: Adjust selected parameter
        double increment = stepSizes[stepIndex]; // Use current step size

        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (tuningShooter) {
                switch (selectedPIDParameter) {
                    case 0: shooter_kP += increment; break;
                    case 1: shooter_kI += increment; break;
                    case 2: shooter_kD += increment; break;
                    case 3: shooter_kF += increment; break;
                }
            } else {
                switch (selectedPIDParameter) {
                    case 0: spindexer_kP += increment; break;
                    case 1: spindexer_kI += increment; break;
                    case 2: spindexer_kD += increment; break;
                    case 3: spindexer_kF += increment; break;
                }
            }
            updatePIDController();
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (tuningShooter) {
                switch (selectedPIDParameter) {
                    case 0: shooter_kP = Math.max(0, shooter_kP - increment); break;
                    case 1: shooter_kI = Math.max(0, shooter_kI - increment); break;
                    case 2: shooter_kD = Math.max(0, shooter_kD - increment); break;
                    case 3: shooter_kF = Math.max(0, shooter_kF - increment); break;
                }
            } else {
                switch (selectedPIDParameter) {
                    case 0: spindexer_kP = Math.max(0, spindexer_kP - increment); break;
                    case 1: spindexer_kI = Math.max(0, spindexer_kI - increment); break;
                    case 2: spindexer_kD = Math.max(0, spindexer_kD - increment); break;
                    case 3: spindexer_kF = Math.max(0, spindexer_kF - increment); break;
                }
            }
            updatePIDController();
        }
        dpadDownPressed = gamepad1.dpad_down;
    }

    /**
     * Apply current PID values to the appropriate controller
     */
    private void updatePIDController() {
        if (tuningShooter) {
            // Apply to shooter motors
            com.qualcomm.robotcore.hardware.PIDFCoefficients pidCoefficients =
                    new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                            shooter_kP, shooter_kI, shooter_kD, shooter_kF);
            robot.shooterMotor1.setPIDFCoefficients(
                    com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                    pidCoefficients);
            robot.shooterMotor2.setPIDFCoefficients(
                    com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                    pidCoefficients);
        } else {
            // Apply to spindexer
            spindexer.setSpindexerPIDF(spindexer_kP, spindexer_kI, spindexer_kD, spindexer_kF);
        }
    }

    private void updateSubsystems() {
        spindexer.periodic();
        shooter.periodic();
        sequenceManager.update();
        catalogManager.update();



    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;

        // ========================================
        // 1. PID TUNING MODE DISPLAY (if active)
        // ========================================
        if (pidTuningMode) {
            telemetry.addLine("========================================");
            if (tuningShooter) {
                telemetry.addLine("🔧 SHOOTER PIDF TUNING MODE ACTIVE 🔧");
                telemetry.addLine("========================================");
                telemetry.addData("Target Velocity:", "%.0f ticks/sec",
                        robot.shooterMotor1.getVelocity());
                telemetry.addData("Motor 1 Velocity:", "%.0f", robot.shooterMotor1.getVelocity());
                telemetry.addData("Motor 2 Velocity:", "%.0f", robot.shooterMotor2.getVelocity());
                telemetry.addData("Velocity Error:", "%.0f",
                        2200.0 - (robot.shooterMotor1.getVelocity() + robot.shooterMotor2.getVelocity()) / 2.0);

                String[] paramNames = {"kP", "kI", "kD", "kF"};
                double[] paramValues = {shooter_kP, shooter_kI, shooter_kD, shooter_kF};

                for (int i = 0; i < 4; i++) {
                    String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                    String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                    telemetry.addData(prefix + paramNames[i] + suffix,
                            String.format("%.3f", paramValues[i]));
                }

                telemetry.addLine("----------------------------------------");
                telemetry.addData("Step Size", "%.3f (Press B to change)", stepSizes[stepIndex]);
                telemetry.addLine("D-Pad Left/Right: Select parameter");
                telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.3f)", stepSizes[stepIndex]));
                telemetry.addLine("Y Button: Switch to Spindexer tuning");
                telemetry.addLine("Back Button: Exit tuning mode");
                telemetry.addLine("========================================");
                telemetry.addLine("Current PIDF Constants:");
                telemetry.addData("", "SHOOTER_P = %.3f", shooter_kP);
                telemetry.addData("", "SHOOTER_I = %.3f", shooter_kI);
                telemetry.addData("", "SHOOTER_D = %.3f", shooter_kD);
                telemetry.addData("", "SHOOTER_F = %.3f", shooter_kF);
            } else {
                telemetry.addLine("🔧 SPINDEXER PIDF TUNING MODE ACTIVE 🔧");
                telemetry.addLine("========================================");
                telemetry.addData("Position Error:", String.format("%.1f°",
                        spindexer.getTargetPosition() - spindexer.getServoPosition()));

                String[] paramNames = {"kP", "kI", "kD", "kF"};
                double[] paramValues = {spindexer_kP, spindexer_kI, spindexer_kD, spindexer_kF};

                for (int i = 0; i < 4; i++) {
                    String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                    String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                    telemetry.addData(prefix + paramNames[i] + suffix,
                            String.format("%.6f", paramValues[i]));
                }

                telemetry.addLine("----------------------------------------");
                telemetry.addData("Step Size", "%.6f (Press B to change)", stepSizes[stepIndex]);
                telemetry.addLine("D-Pad Left/Right: Select parameter");
                telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.6f)", stepSizes[stepIndex]));
                telemetry.addLine("Y Button: Switch to Shooter tuning");
                telemetry.addLine("Back Button: Exit tuning mode");
                telemetry.addLine("========================================");
                telemetry.addLine("Current PIDF Constants:");
                telemetry.addData("", "SPINDEXER_P = %.5f", spindexer_kP);
                telemetry.addData("", "SPINDEXER_I = %.5f", spindexer_kI);
                telemetry.addData("", "SPINDEXER_D = %.5f", spindexer_kD);
                telemetry.addData("", "SPINDEXER_F = %.5f", spindexer_kF);
            }
            telemetry.addLine("========================================");
            telemetry.addLine("");
        }

        // ========================================
        // 2. TOP-LEVEL KEY INFORMATION
        // ========================================
        telemetry.addData("Tracking Mode", limelight.getCurrentMode());
        telemetry.addData("Spindexer Pattern", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());

        // Motif Pattern with fallback
        if (limelight.isMotifDetected()) {
            telemetry.addData("Motif Pattern", String.format("[%s, %s, %s]",
                    RobotConstants.Limelight.motifPattern.getBallColorInSlotX(0),
                    RobotConstants.Limelight.motifPattern.getBallColorInSlotX(1),
                    RobotConstants.Limelight.motifPattern.getBallColorInSlotX(2)));
        } else {
            telemetry.addData("Motif Pattern", "Not Detected");
        }

        telemetry.addData("Spindexer Error", String.format("%.1f°",
                spindexer.getTargetPosition() - spindexer.getServoPosition()));
        telemetry.addData("Turret Error", String.format("%.1f°",
                shooter.getTargetTurretAngle() - shooter.getTurretPosition()));
        telemetry.addLine("----------------------------------------");

        // ========================================
        // 3. ORGANIZED SUBSYSTEMS
        // ========================================

        // SUBSYSTEMS section
        telemetry.addLine("=== SUBSYSTEMS ===");
        telemetry.addData("  Drive State", mecanumDrive.getCurrentState());
        telemetry.addData("  Intake State", intake.getCurrentState());
        telemetry.addData("  Shooter State", shooter.getCurrentState());
        telemetry.addData("  Spindexer State", spindexer.getCurrentState());
        telemetry.addLine("");

        // SHOOTING section
        telemetry.addLine("=== SHOOTING ===");
        telemetry.addData("  Field Zone", shooter.getFieldState());
        telemetry.addData("  Shooting Status", shootingValidator.getStatus(overrideRequested));
        telemetry.addData("  Executing Sequence", sequenceManager.isExecuting());
        if (sequenceManager.isExecuting()) {
            telemetry.addData("  Sequence Status", sequenceManager.getStatus());
        }
        telemetry.addLine("");

        // SHOOTER section
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("  Shooter Velocity", String.format("%.0f", robot.shooterMotor2.getVelocity()));
        telemetry.addData("  Distance to Target", shooter.getDistanceToTarget());
        telemetry.addData("  Turret Position", String.format("%.1f°", shooter.getTurretPosition()));
        telemetry.addData("  Turret Target", String.format("%.1f°", shooter.getTargetTurretAngle()));
        telemetry.addLine("");

        // SPINDEXER section
        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("  Current Position", String.format("%.1f°", spindexer.getServoPosition()));
        telemetry.addData("  Target Position", String.format("%d°", spindexer.getTargetPosition()));
        telemetry.addData("  Slot Index", spindexer.getSpindPosTracker());
        telemetry.addData("  Rotation State", spindexer.getRotationState());
        telemetry.addLine("");

        // CATALOGING section
        telemetry.addLine("=== CATALOGING ===");
        telemetry.addData("  Catalog State", this.catalogManager.getState());
        DualBallDetector.Result result = robot.intakeSensorPair.detectBall();
        telemetry.addData("  Ball Present", result.ballPresent);
        telemetry.addData("  Ball Color", result.color);
        telemetry.addData("  Confidence", String.format("%.1f%%", result.confidence * 100));
        telemetry.addLine("");

        // OTHER section
        telemetry.addLine("=== OTHER ===");
        telemetry.addData("  Alliance", RobotConstants.UpdatableConstants.allianceColor);
        telemetry.addLine("");

        // SEQUENCE DEBUG section (only when sequence executing)
        if (sequenceManager.isExecuting()) {
            telemetry.addLine("=== SEQUENCE DEBUG ===");
            telemetry.addData("  Done Rotating", spindexer.isDoneRotating());
            telemetry.addData("  Ready to Flip", spindexer.isReadyToFlip());
            telemetry.addLine("");
        }

        telemetry.update();
    }
}