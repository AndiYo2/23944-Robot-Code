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

/**
 * TeleOp Template WITH PID Tuning Capabilities
 *
 * This template includes full PID tuning functionality for Spindexer, Shooter, and Turret.
 * Use this template when you need to tune PIDF parameters during testing.
 *
 * For competition without tuning controls, use TeleOpTemplate instead.
 */
abstract public class TeleOpTemplateTuning extends CommandOpMode {
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
    private int tuningSubsystem = 0; // 0=spindexer, 1=shooter, 2=turret
    private int selectedPIDParameter = 0; // 0=P, 1=I, 2=D, 3=F
    private boolean tuningSpindexerCW = true; // true = CW PIDs, false = CCW PIDs
    private boolean adjustingPower = false; // false = adjust PIDF, true = adjust power/velocity

    // Spindexer PIDF values - CW rotation (with gravity)
    private double spindexer_cw_kP, spindexer_cw_kI, spindexer_cw_kD, spindexer_cw_kF;

    // Spindexer PIDF values - CCW rotation (against gravity)
    private double spindexer_ccw_kP, spindexer_ccw_kI, spindexer_ccw_kD, spindexer_ccw_kF;

    // Shooter PIDF values
    private double shooter_kP, shooter_kI, shooter_kD, shooter_kF;

    // Turret PID values
    private double turret_kP, turret_kI, turret_kD;

    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;
    private boolean dpadLeftPressed = false;
    private boolean dpadRightPressed = false;
    private boolean bButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean leftStickButtonPressed = false;

    // Step sizes for tuning - press B (in tuning mode) to cycle
    private double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001, 0.000001, 0.0000001};
    private int stepIndex = 3; // Start with 0.01

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        // Set starting position for TeleOp
        // If we have an ending auton pose, use it; otherwise use standard start point
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D startPosition;
        if (RobotConstants.UpdatableConstants.endingAutonPose != null) {
            // Convert Pedro Pose to FTC Pose2D
            com.pedropathing.geometry.Pose autonPose = RobotConstants.UpdatableConstants.endingAutonPose;
            startPosition = new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                    autonPose.getX(),
                    autonPose.getY(),
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                    autonPose.getHeading());
            telemetry.addData("TeleOp Init", "Using Auton End Position");
        } else {
            // No auton ran - use standard starting position
            startPosition = RobotConstants.Pinpoint.standardStartPoint;
            telemetry.addData("TeleOp Init", "Using Standard Start Position");
        }

        telemetry.addData("Setting Position To", startPosition);
        telemetry.update();

        robot.pinpoint.setPosition(startPosition);
        // CRITICAL: Update Pinpoint after setting position to apply it
        robot.pinpoint.update();

        telemetry.addData("Position After Set", robot.pinpoint.getPosition());
        telemetry.update();

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
        // Spindexer CW PIDs (with gravity)
        spindexer_cw_kP = RobotConstants.Spindexer.SPINDEXER_CW_P;
        spindexer_cw_kI = RobotConstants.Spindexer.SPINDEXER_CW_I;
        spindexer_cw_kD = RobotConstants.Spindexer.SPINDEXER_CW_D;
        spindexer_cw_kF = RobotConstants.Spindexer.SPINDEXER_CW_F;

        // Spindexer CCW PIDs (against gravity)
        spindexer_ccw_kP = RobotConstants.Spindexer.SPINDEXER_CCW_P;
        spindexer_ccw_kI = RobotConstants.Spindexer.SPINDEXER_CCW_I;
        spindexer_ccw_kD = RobotConstants.Spindexer.SPINDEXER_CCW_D;
        spindexer_ccw_kF = RobotConstants.Spindexer.SPINDEXER_CCW_F;

        // Shooter PIDs
        shooter_kP = RobotConstants.ShooterPIDF.P;
        shooter_kI = RobotConstants.ShooterPIDF.I;
        shooter_kD = RobotConstants.ShooterPIDF.D;
        shooter_kF = RobotConstants.ShooterPIDF.F;

        // Turret PIDs
        turret_kP = RobotConstants.Shooter.TURRET_PID.p;
        turret_kI = RobotConstants.Shooter.TURRET_PID.i;
        turret_kD = RobotConstants.Shooter.TURRET_PID.d;

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
                .whenPressed(() -> {
                    if (!pidTuningMode || tuningSubsystem != 1) mecanumDrive.toggleSlowMode();
                });
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.triggerFlick());
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> attemptShootingSequence());
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    // Allow cataloging in shooter tuning mode (subsystem 1) for full robot functionality
                    if (!pidTuningMode || tuningSubsystem == 1) catalogManager.initiateCataloging();
                });

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> manualRotateCCW());

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(() -> manualRotateCW());

        // PID Tuning Mode cycle (Back button cycles through all modes)
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)
                .whenPressed(() -> cycleTuningMode());

        // Mode toggles (when NOT in tuning mode)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> {
                    if (!pidTuningMode) {
                        limelight.toggleMode();
                    }
                });
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> {
                    if (!pidTuningMode) {
                        limelight.resetLimelight();
                    }
                });
    }

    @Override
    public void run() {
        super.run();

        // CRITICAL: Update Pinpoint odometry every loop (like in test OpMode)
        robot.pinpoint.update();

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
     * Use SWAP_ALLIANCE_CONTROLS in RobotConstants to swap which alliance gets inverted controls.
     *
     * @return double array [fieldY, fieldX, rotation]
     */
    private double[] getTransformedControls() {
        double rawY = -gamepad1.left_stick_y;  // Negative because gamepad Y is inverted
        double rawX = gamepad1.left_stick_x;
        double rawRotation = gamepad1.right_stick_x;

        // Determine which alliance should have inverted controls
        RobotConstants.Enums.AllianceColor invertedAlliance = RobotConstants.Controls.SWAP_ALLIANCE_CONTROLS
                ? RobotConstants.Enums.AllianceColor.Blue
                : RobotConstants.Enums.AllianceColor.Red;

        // Check alliance color (defaults to Blue if not set)
        if (RobotConstants.UpdatableConstants.allianceColor != null &&
                RobotConstants.UpdatableConstants.allianceColor == invertedAlliance) {
            // Invert both translational axes (180° field rotation)
            return new double[] {-rawY, -rawX, rawRotation};
        } else {
            // Standard mapping
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
     * OVERRIDE: Hold left stick button to force rotation during sequence
     */
    private void manualRotateCW() {
        boolean forceOverride = gamepad1.left_stick_button;
        if (sequenceManager.isExecuting() && !forceOverride) {
            return; // Block manual rotation during shooting sequence (unless override held)
        }
        spindexer.rotateCW();
    }

    /**
     * Manual rotation backward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     * OVERRIDE: Hold left stick button to force rotation during sequence
     */
    private void manualRotateCCW() {
        boolean forceOverride = gamepad1.left_stick_button;
        if (sequenceManager.isExecuting() && !forceOverride) {
            return; // Block manual rotation during shooting sequence (unless override held)
        }
        spindexer.rotateCCW();
    }

    /**
     * Cycle through all PID tuning modes:
     * OFF → Spindexer → Shooter PIDF → Shooter Power → Turret → OFF
     */
    private void cycleTuningMode() {
        if (!pidTuningMode) {
            // Currently OFF, enable and start with Spindexer (subsystem 0)
            pidTuningMode = true;
            tuningSubsystem = 0;
            adjustingPower = false;
            gamepad1.rumble(100); // Short rumble
            shooter.setManualVelocityMode(false);
        } else {
            // Currently ON, cycle to next mode
            if (tuningSubsystem == 0) {
                // Spindexer → Shooter Tuning (conjoined PIDF + Velocity)
                tuningSubsystem = 1;
                adjustingPower = false; // Start in PIDF adjust mode (d-pad adjusts PIDF)
                // Enable manual velocity mode so tuning velocity is used (not distance-based)
                shooter.setManualVelocityMode(true);
                shooter.setManualVelocity(RobotConstants.ShooterPIDF.TUNING_VELOCITY);
                gamepad1.rumble(100);
            } else if (tuningSubsystem == 1) {
                // Shooter Tuning → Turret PID (skip separate velocity mode, it's conjoined)
                tuningSubsystem = 2;
                adjustingPower = false;
                shooter.setManualVelocityMode(false); // Exit shooter tuning mode
                gamepad1.rumble(100);
            } else if (tuningSubsystem == 2 && !adjustingPower) {
                // Turret PID → Turret Target
                adjustingPower = true;
                gamepad1.rumble(100);
            } else if (tuningSubsystem == 2 && adjustingPower) {
                // Turret Target → OFF
                pidTuningMode = false;
                tuningSubsystem = 0;
                adjustingPower = false;
                shooter.setManualVelocityMode(false);
                gamepad1.rumble(500); // Long rumble for OFF
            }
        }
    }

    /**
     * Handle PID tuning controls when tuning mode is active
     */
    private void updatePIDTuning() {
        if (!pidTuningMode) return;

        // Sync configurable values from RobotConstants (Panels updates these automatically)
        if (tuningSubsystem == 1) {
            // Sync local tuning variables with RobotConstants (Panels pushes updates automatically)
            shooter_kP = RobotConstants.ShooterPIDF.P;
            shooter_kI = RobotConstants.ShooterPIDF.I;
            shooter_kD = RobotConstants.ShooterPIDF.D;
            shooter_kF = RobotConstants.ShooterPIDF.F;

            // Always apply tuning velocity (conjoined tuning - PIDF and velocity together)
            shooter.setManualVelocity(RobotConstants.ShooterPIDF.TUNING_VELOCITY);
        } else if (tuningSubsystem == 2) {
            // Keep turret target synced with configurable
            shooter.setTurretDegree(RobotConstants.Shooter.TURRET_TUNING_TARGET);
        }

        // Left thumbstick button: Toggle d-pad mode between PIDF adjust and Velocity adjust
        if (gamepad1.left_stick_button && !leftStickButtonPressed) {
            adjustingPower = !adjustingPower;
            // Shooter tuning: manual velocity mode stays ON, this just changes what d-pad adjusts
            gamepad1.rumble(adjustingPower ? 200 : 100); // Longer rumble for velocity adjust mode
        }
        leftStickButtonPressed = gamepad1.left_stick_button;

        // X button: Toggle between CW and CCW PIDs (when tuning spindexer)
        if (gamepad1.x && !xButtonPressed) {
            if (tuningSubsystem == 0) { // Spindexer
                tuningSpindexerCW = !tuningSpindexerCW;
                gamepad1.rumble(100); // Rumble to confirm switch
            }
            // X now always triggers spindexer flick (handled in configureButtonBindings)
        }
        xButtonPressed = gamepad1.x;

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

        // D-pad up/down: Adjust selected parameter OR power/velocity/target
        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (adjustingPower && tuningSubsystem == 1) {
                // Power mode: adjust shooter velocity by step size
                RobotConstants.ShooterPIDF.TUNING_VELOCITY += stepSizes[stepIndex];
                shooter.setManualVelocity(RobotConstants.ShooterPIDF.TUNING_VELOCITY);
                gamepad1.rumble(50);
            } else if (adjustingPower && tuningSubsystem == 2) {
                // Target mode: adjust turret target angle by step size
                RobotConstants.Shooter.TURRET_TUNING_TARGET += stepSizes[stepIndex];
                gamepad1.rumble(50);
            } else {
                // PIDF mode: adjust selected PIDF parameter
                double increment = stepSizes[stepIndex]; // Use current step size
                switch (tuningSubsystem) {
                    case 1: // Shooter
                        switch (selectedPIDParameter) {
                            case 0: shooter_kP += increment; break;
                            case 1: shooter_kI += increment; break;
                            case 2: shooter_kD += increment; break;
                            case 3: shooter_kF += increment; break;
                        }
                        break;
                    case 2: // Turret
                        switch (selectedPIDParameter) {
                            case 0: turret_kP += increment; break;
                            case 1: turret_kI += increment; break;
                            case 2: turret_kD += increment; break;
                        }
                        break;
                    default: // Spindexer
                        if (tuningSpindexerCW) {
                            switch (selectedPIDParameter) {
                                case 0: spindexer_cw_kP += increment; break;
                                case 1: spindexer_cw_kI += increment; break;
                                case 2: spindexer_cw_kD += increment; break;
                                case 3: spindexer_cw_kF += increment; break;
                            }
                        } else {
                            switch (selectedPIDParameter) {
                                case 0: spindexer_ccw_kP += increment; break;
                                case 1: spindexer_ccw_kI += increment; break;
                                case 2: spindexer_ccw_kD += increment; break;
                                case 3: spindexer_ccw_kF += increment; break;
                            }
                        }
                        break;
                }
                updatePIDController();
            }
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            if (adjustingPower && tuningSubsystem == 1) {
                // Power mode: adjust shooter velocity by step size
                RobotConstants.ShooterPIDF.TUNING_VELOCITY -= stepSizes[stepIndex];
                shooter.setManualVelocity(RobotConstants.ShooterPIDF.TUNING_VELOCITY);
                gamepad1.rumble(50);
            } else if (adjustingPower && tuningSubsystem == 2) {
                // Target mode: adjust turret target angle by step size
                RobotConstants.Shooter.TURRET_TUNING_TARGET -= stepSizes[stepIndex];
                gamepad1.rumble(50);
            } else {
                // PIDF mode: adjust selected PIDF parameter
                double increment = stepSizes[stepIndex]; // Use current step size
                switch (tuningSubsystem) {
                    case 1: // Shooter
                        switch (selectedPIDParameter) {
                            case 0: shooter_kP = Math.max(0, shooter_kP - increment); break;
                            case 1: shooter_kI = Math.max(0, shooter_kI - increment); break;
                            case 2: shooter_kD = Math.max(0, shooter_kD - increment); break;
                            case 3: shooter_kF = Math.max(0, shooter_kF - increment); break;
                        }
                        break;
                    case 2: // Turret
                        switch (selectedPIDParameter) {
                            case 0: turret_kP = Math.max(0, turret_kP - increment); break;
                            case 1: turret_kI = Math.max(0, turret_kI - increment); break;
                            case 2: turret_kD = Math.max(0, turret_kD - increment); break;
                        }
                        break;
                    default: // Spindexer
                        if (tuningSpindexerCW) {
                            switch (selectedPIDParameter) {
                                case 0: spindexer_cw_kP = Math.max(0, spindexer_cw_kP - increment); break;
                                case 1: spindexer_cw_kI = Math.max(0, spindexer_cw_kI - increment); break;
                                case 2: spindexer_cw_kD = Math.max(0, spindexer_cw_kD - increment); break;
                                case 3: spindexer_cw_kF = Math.max(0, spindexer_cw_kF - increment); break;
                            }
                        } else {
                            switch (selectedPIDParameter) {
                                case 0: spindexer_ccw_kP = Math.max(0, spindexer_ccw_kP - increment); break;
                                case 1: spindexer_ccw_kI = Math.max(0, spindexer_ccw_kI - increment); break;
                                case 2: spindexer_ccw_kD = Math.max(0, spindexer_ccw_kD - increment); break;
                                case 3: spindexer_ccw_kF = Math.max(0, spindexer_ccw_kF - increment); break;
                            }
                        }
                        break;
                }
                updatePIDController();
            }
        }
        dpadDownPressed = gamepad1.dpad_down;
    }

    /**
     * Apply current PID values to the appropriate controller
     */
    private void updatePIDController() {
        switch (tuningSubsystem) {
            case 1: // Shooter
                // Update RobotConstants so Shooter.periodic() picks up the new values
                RobotConstants.ShooterPIDF.P = shooter_kP;
                RobotConstants.ShooterPIDF.I = shooter_kI;
                RobotConstants.ShooterPIDF.D = shooter_kD;
                RobotConstants.ShooterPIDF.F = shooter_kF;
                break;

            case 2: // Turret
                shooter.setTurretPID(turret_kP, turret_kI, turret_kD);
                break;

            default: // Spindexer
                if (tuningSpindexerCW) {
                    spindexer.setCWPIDF(spindexer_cw_kP, spindexer_cw_kI, spindexer_cw_kD, spindexer_cw_kF);
                } else {
                    spindexer.setCCWPIDF(spindexer_ccw_kP, spindexer_ccw_kI, spindexer_ccw_kD, spindexer_ccw_kF);
                }
                break;
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
            switch (tuningSubsystem) {
                case 1: // Shooter
                    telemetry.addLine("🔧 SHOOTER PIDF TUNING MODE ACTIVE 🔧");
                    telemetry.addLine("========================================");

                // Calculate velocities and errors
                double targetVelocity = shooter.getManualVelocity();
                double motor1Velocity = robot.shooterMotor1.getVelocity();
                double motor2Velocity = robot.shooterMotor2.getVelocity();
                double motor1Power = robot.shooterMotor1.getPower();
                double motor2Power = robot.shooterMotor2.getPower();
                int motor1EncoderPos = robot.shooterMotor1.getCurrentPosition();
                int motor2EncoderPos = robot.shooterMotor2.getCurrentPosition();
                double velocityError = targetVelocity - motor2Velocity;
                double percentError = (targetVelocity > 0) ? (velocityError / targetVelocity) * 100.0 : 0;

                // Velocity Information
                telemetry.addLine("--- VELOCITY ---");
                telemetry.addData("Target Velocity", "%.0f ticks/sec", targetVelocity);
                telemetry.addData("Motor 1 Velocity", "%.0f ticks/sec", motor1Velocity);
                telemetry.addData("Motor 2 Velocity", "%.0f ticks/sec", motor2Velocity);
                telemetry.addData("Velocity Error", "%.0f ticks/sec (%.1f%%)", velocityError, percentError);
                telemetry.addLine("");
                telemetry.addData("Motor 1 Encoder Pos", motor1EncoderPos);
                telemetry.addData("Motor 2 Encoder Pos", motor2EncoderPos);
                telemetry.addData("Motor 1 Power", "%.3f", motor1Power);
                telemetry.addData("Motor 2 Power", "%.3f", motor2Power);

                telemetry.addLine("");
                telemetry.addLine("--- PIDF PARAMETERS ---");
                String[] paramNames = {"kP", "kI", "kD", "kF"};
                double[] paramValues = {shooter_kP, shooter_kI, shooter_kD, shooter_kF};

                for (int i = 0; i < 4; i++) {
                    String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                    String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                    telemetry.addData(prefix + paramNames[i] + suffix,
                            String.format("%.3f", paramValues[i]));
                }

                telemetry.addLine("----------------------------------------");
                telemetry.addData("Tuning Velocity", "%.0f ticks/sec", RobotConstants.ShooterPIDF.TUNING_VELOCITY);
                telemetry.addLine("");
                if (adjustingPower) {
                    telemetry.addData("D-Pad Mode", "VELOCITY ADJUST");
                    telemetry.addData("Step Size", "%.1f (Press B to change)", stepSizes[stepIndex]);
                    telemetry.addLine("D-Pad Up/Down: Adjust velocity");
                    telemetry.addLine("Left Stick Button: Switch to PIDF adjust");
                } else {
                    telemetry.addData("D-Pad Mode", "PIDF ADJUST");
                    telemetry.addData("Step Size", "%.3f (Press B to change)", stepSizes[stepIndex]);
                    telemetry.addLine("D-Pad Left/Right: Select parameter");
                    telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.3f)", stepSizes[stepIndex]));
                    telemetry.addLine("Left Stick Button: Switch to VELOCITY adjust");
                }
                telemetry.addLine("");
                telemetry.addLine("*** CONJOINED TUNING - Both PIDF & Velocity active ***");
                telemetry.addLine("Panels updates: P, I, D, F, TUNING_VELOCITY");
                telemetry.addLine("Left/Right Bumpers: Manual spindexer rotation");
                telemetry.addLine("X Button: Trigger spindexer flick");
                telemetry.addLine("Back Button: Cycle tuning mode (→Turret→OFF)");
                telemetry.addLine("========================================");
                telemetry.addLine("Current Constants:");
                telemetry.addData("", "SHOOTER_P = %.3f", shooter_kP);
                telemetry.addData("", "SHOOTER_I = %.3f", shooter_kI);
                telemetry.addData("", "SHOOTER_D = %.3f", shooter_kD);
                telemetry.addData("", "SHOOTER_F = %.3f", shooter_kF);
                telemetry.addData("", "SHOOTER_TUNING_VELOCITY = %.0f", RobotConstants.ShooterPIDF.TUNING_VELOCITY);
                    break;

                case 2: // Turret
                    telemetry.addLine("🔧 TURRET TUNING MODE ACTIVE 🔧");
                    telemetry.addLine("========================================");

                    // Calculate turret errors
                    double turretTarget = shooter.getTargetTurretAngle();
                    double turretPosition = shooter.getTurretPosition();
                    double turretError = turretTarget - turretPosition;

                    // Turret Information
                    telemetry.addLine("--- TURRET POSITION ---");
                    telemetry.addData("Tuning Target (turret deg)", "%.1f°", RobotConstants.Shooter.TURRET_TUNING_TARGET);
                    telemetry.addData("Actual Target (servo deg)", "%.2f°", turretTarget);
                    telemetry.addData("Current Position (servo deg)", "%.2f°", turretPosition);
                    telemetry.addData("Position Error", "%.2f°", turretError);
                    telemetry.addLine("");

                    if (adjustingPower) {
                        // Target adjustment mode
                        telemetry.addLine(">>> TARGET ADJUSTMENT MODE <<<");
                        telemetry.addData("Tuning Target", "%.1f° (turret degrees)", RobotConstants.Shooter.TURRET_TUNING_TARGET);
                        telemetry.addLine("");
                        telemetry.addLine("----------------------------------------");
                        telemetry.addData("Step Size", "%.2f (Press B to change)", stepSizes[stepIndex]);
                        telemetry.addLine("D-Pad Up/Down: Adjust target angle");
                        telemetry.addLine("Left Stick Button: Switch to PID mode");
                    } else {
                        // PID adjustment mode
                        telemetry.addLine("--- PID PARAMETERS ---");
                        String[] turretParamNames = {"kP", "kI", "kD"};
                        double[] turretParamValues = {turret_kP, turret_kI, turret_kD};

                        for (int i = 0; i < 3; i++) {
                            String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                            String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                            telemetry.addData(prefix + turretParamNames[i] + suffix,
                                    String.format("%.5f", turretParamValues[i]));
                        }

                        telemetry.addLine("----------------------------------------");
                        telemetry.addData("Step Size", "%.5f (Press B to change)", stepSizes[stepIndex]);
                        telemetry.addLine("D-Pad Left/Right: Select parameter");
                        telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.5f)", stepSizes[stepIndex]));
                        telemetry.addLine("Left Stick Button: Switch to TARGET mode");
                    }
                    telemetry.addLine("Back Button: Cycle tuning mode (→OFF→Spindexer→Shooter)");
                    telemetry.addLine("========================================");
                    telemetry.addLine("Current PID Constants:");
                    telemetry.addData("", "TURRET_P = %.5f", turret_kP);
                    telemetry.addData("", "TURRET_I = %.5f", turret_kI);
                    telemetry.addData("", "TURRET_D = %.5f", turret_kD);
                    telemetry.addData("", "TURRET_TUNING_TARGET = %.1f", RobotConstants.Shooter.TURRET_TUNING_TARGET);
                    break;

                default: // Spindexer
                String pidType = tuningSpindexerCW ? "CW (with gravity)" : "CCW (against gravity)";
                telemetry.addLine("🔧 SPINDEXER PIDF TUNING MODE ACTIVE 🔧");
                telemetry.addLine("========================================");
                telemetry.addData("Tuning PID Set:", pidType);
                telemetry.addData("Position Error:", String.format("%.1f°",
                        spindexer.getTargetPosition() - spindexer.getServoPosition()));

                String[] spindexerParamNames = {"kP", "kI", "kD", "kF"};
                double[] spindexerParamValues;

                if (tuningSpindexerCW) {
                    spindexerParamValues = new double[]{spindexer_cw_kP, spindexer_cw_kI, spindexer_cw_kD, spindexer_cw_kF};
                } else {
                    spindexerParamValues = new double[]{spindexer_ccw_kP, spindexer_ccw_kI, spindexer_ccw_kD, spindexer_ccw_kF};
                }

                for (int i = 0; i < 4; i++) {
                    String prefix = (i == selectedPIDParameter) ? ">>> " : "    ";
                    String suffix = (i == selectedPIDParameter) ? " <<<" : "";
                    telemetry.addData(prefix + spindexerParamNames[i] + suffix,
                            String.format("%.6f", spindexerParamValues[i]));
                }

                telemetry.addLine("----------------------------------------");
                telemetry.addData("Step Size", "%.6f (Press B to change)", stepSizes[stepIndex]);
                telemetry.addLine("D-Pad Left/Right: Select parameter");
                telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.6f)", stepSizes[stepIndex]));
                telemetry.addLine("X Button: Toggle CW/CCW PID");
                telemetry.addLine("Back Button: Cycle tuning mode (→Shooter→Turret→OFF)");
                telemetry.addLine("========================================");
                telemetry.addLine("Current PIDF Constants:");
                if (tuningSpindexerCW) {
                    telemetry.addData("", "SPINDEXER_CW_P = %.5f", spindexer_cw_kP);
                    telemetry.addData("", "SPINDEXER_CW_I = %.5f", spindexer_cw_kI);
                    telemetry.addData("", "SPINDEXER_CW_D = %.5f", spindexer_cw_kD);
                    telemetry.addData("", "SPINDEXER_CW_F = %.5f", spindexer_cw_kF);
                } else {
                    telemetry.addData("", "SPINDEXER_CCW_P = %.5f", spindexer_ccw_kP);
                    telemetry.addData("", "SPINDEXER_CCW_I = %.5f", spindexer_ccw_kI);
                    telemetry.addData("", "SPINDEXER_CCW_D = %.5f", spindexer_ccw_kD);
                    telemetry.addData("", "SPINDEXER_CCW_F = %.5f", spindexer_ccw_kF);
                }
                // Show ACTUAL values in the PID controller (for debugging)
                double[] actualPIDF = spindexer.getActivePIDFCoefficients();
                telemetry.addLine("--- ACTUAL PID Controller Values ---");
                telemetry.addData("  Active P", "%.5f", actualPIDF[0]);
                telemetry.addData("  Active I", "%.5f", actualPIDF[1]);
                telemetry.addData("  Active D", "%.5f", actualPIDF[2]);
                telemetry.addData("  Active F", "%.5f", actualPIDF[3]);
                telemetry.addLine("--- PID OUTPUT DEBUG ---");
                telemetry.addData("  Error (deg)", "%.2f", spindexer.getLastError());
                telemetry.addData("  PID Output", "%.4f", spindexer.getLastPIDOutput());
                telemetry.addData("  Servo Power", "%.4f", robot.spindexerServo.getPower());
                    break;
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

        // ODOMETRY section
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("  X Position", String.format("%.1f in", robot.pinpoint.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("  Y Position", String.format("%.1f in", robot.pinpoint.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("  Heading", String.format("%.3f rad", robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)));
        telemetry.addLine("");

        // OTHER section
        telemetry.addLine("=== OTHER ===");
        telemetry.addData("  Alliance", RobotConstants.UpdatableConstants.allianceColor);
        telemetry.addData("  IMU Calibration", "Auto on init (resetPosAndIMU)");
        telemetry.addLine("");

        // SEQUENCE DEBUG section (only when sequence executing)
        if (sequenceManager.isExecuting()) {
            telemetry.addLine("=== SEQUENCE DEBUG ===");
            telemetry.addData("  Done Rotating", spindexer.isDoneRotating());
            telemetry.addData("  Ready to Flip", spindexer.isReadyToFlip());
            telemetry.addLine("");
        }

        // ========================================
        // TURRET TRACKING DEBUG (for visualizer comparison)
        // ========================================
        telemetry.addLine("=== TURRET TRACKING DEBUG ===");

        // Robot Position (for visualizer comparison)
        double robotX = robot.pinpoint.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotY = robot.pinpoint.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH);
        double robotHeadingRad = robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS);
        double robotHeadingDeg = Math.toDegrees(robotHeadingRad);

        telemetry.addData("  Robot Pose", String.format("(%.1f, %.1f, %.1f°)", robotX, robotY, robotHeadingDeg));

        // Goal Position
        com.pedropathing.geometry.Pose goalPos = FieldMap.getGoalPosition();
        telemetry.addData("  Goal Position", String.format("(%.0f, %.0f)", goalPos.getX(), goalPos.getY()));

        // Delta to goal (from robot center)
        double deltaX = goalPos.getX() - robotX;
        double deltaY = goalPos.getY() - robotY;
        telemetry.addData("  Delta to Goal", String.format("dx=%.1f, dy=%.1f", deltaX, deltaY));

        // Field angle to goal
        double fieldAngleRad = Math.atan2(deltaY, deltaX);
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
        telemetry.addData("  Field Angle to Goal", String.format("%.1f°", fieldAngleDeg));

        // Expected turret angle (field angle - robot heading)
        double expectedTurretAngle = fieldAngleDeg - robotHeadingDeg;
        // Normalize to [-180, 180]
        while (expectedTurretAngle > 180) expectedTurretAngle -= 360;
        while (expectedTurretAngle < -180) expectedTurretAngle += 360;
        telemetry.addData("  Expected Turret Angle", String.format("%.1f°", expectedTurretAngle));

        // Actual turret values
        double turretTarget = shooter.getTargetTurretAngle() / RobotConstants.Shooter.GEAR_RATIO;  // Convert servo deg to turret deg
        double turretActual = shooter.getTurretPosition() / RobotConstants.Shooter.GEAR_RATIO;    // Convert servo deg to turret deg
        double turretError = turretTarget - turretActual;

        telemetry.addData("  Turret Target", String.format("%.1f° (turret deg)", turretTarget));
        telemetry.addData("  Turret Actual", String.format("%.1f° (turret deg)", turretActual));
        telemetry.addData("  Turret Error", String.format("%.1f°", turretError));

        // Comparison: Expected vs What code calculated
        telemetry.addData("  Calc vs Expected Diff", String.format("%.1f°", turretTarget - expectedTurretAngle));

        telemetry.addLine("");

        telemetry.update();
    }
}
