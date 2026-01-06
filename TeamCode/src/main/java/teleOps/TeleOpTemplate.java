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
    private boolean yButtonPressed = false;
    private boolean xButtonPressed = false;
    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;
    private boolean leftStickButtonPressed = false;
    private boolean rightStickButtonPressed = false;

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
        shooter_kP = RobotConstants.Shooter.SHOOTER_P;
        shooter_kI = RobotConstants.Shooter.SHOOTER_I;
        shooter_kD = RobotConstants.Shooter.SHOOTER_D;
        shooter_kF = RobotConstants.Shooter.SHOOTER_F;

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
                    if (!pidTuningMode) mecanumDrive.toggleSlowMode();
                });
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(() -> spindexer.triggerFlick());
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> attemptShootingSequence());
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    if (!pidTuningMode) catalogManager.initiateCataloging();
                });

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
            // If tuning shooter (subsystem 1), enable manual velocity mode
            if (tuningSubsystem == 1) {
                shooter.setManualVelocityMode(true);
            }
        } else {
            gamepad1.rumble(500); // Long rumble on disable
            // Disable manual velocity mode when exiting tuning
            shooter.setManualVelocityMode(false);
        }
    }

    /**
     * Handle PID tuning controls when tuning mode is active
     */
    private void updatePIDTuning() {
        if (!pidTuningMode) return;

        // Y button: Cycle between subsystems (spindexer → shooter → turret)
        if (gamepad1.y && !yButtonPressed) {
            tuningSubsystem = (tuningSubsystem + 1) % 3; // 0=spindexer, 1=shooter, 2=turret
            gamepad1.rumble(100); // Rumble to confirm switch
            // Enable/disable manual velocity mode based on what we're tuning
            if (tuningSubsystem == 1) {
                shooter.setManualVelocityMode(true);
            } else {
                shooter.setManualVelocityMode(false);
            }
        }
        yButtonPressed = gamepad1.y;

        // Right thumbstick button: Jump directly to turret tuning
        if (gamepad1.right_stick_button && !rightStickButtonPressed) {
            tuningSubsystem = 2; // Set to turret
            shooter.setManualVelocityMode(false);
            gamepad1.rumble(150); // Longer rumble for direct selection
        }
        rightStickButtonPressed = gamepad1.right_stick_button;

        // Left thumbstick button: Toggle between PIDF tuning and Power tuning
        if (gamepad1.left_stick_button && !leftStickButtonPressed) {
            adjustingPower = !adjustingPower;
            gamepad1.rumble(adjustingPower ? 200 : 100); // Longer rumble for power mode
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

        // D-pad up/down: Adjust selected parameter OR power/velocity
        if (gamepad1.dpad_up && !dpadUpPressed) {
            if (adjustingPower && tuningSubsystem == 1) {
                // Power mode: adjust shooter velocity by 10
                double currentVelocity = shooter.getManualVelocity();
                shooter.setManualVelocity(currentVelocity + 10);
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
                // Power mode: adjust shooter velocity by -10
                double currentVelocity = shooter.getManualVelocity();
                shooter.setManualVelocity(currentVelocity - 10);
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
                com.qualcomm.robotcore.hardware.PIDFCoefficients pidCoefficients =
                        new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                                shooter_kP, shooter_kI, shooter_kD, shooter_kF);
                robot.shooterMotor1.setPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                        pidCoefficients);
                robot.shooterMotor2.setPIDFCoefficients(
                        com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER,
                        pidCoefficients);
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
                telemetry.addData("Tuning Mode", adjustingPower ? "POWER (±10)" : "PIDF");
                if (!adjustingPower) {
                    telemetry.addData("Step Size", "%.3f (Press B to change)", stepSizes[stepIndex]);
                }
                telemetry.addLine("Left Stick Button: Toggle Power/PIDF mode");
                if (adjustingPower) {
                    telemetry.addLine("D-Pad Up/Down: Adjust velocity (±10)");
                } else {
                    telemetry.addLine("D-Pad Left/Right: Select parameter");
                    telemetry.addLine(String.format("D-Pad Up/Down: Adjust value (±%.3f)", stepSizes[stepIndex]));
                }
                telemetry.addLine("Left/Right Bumpers: Manual spindexer rotation");
                telemetry.addLine("X Button: Trigger spindexer flick");
                telemetry.addLine("Y Button: Cycle subsystem | Right Stick: Jump to Turret");
                telemetry.addLine("Back Button: Exit tuning mode");
                telemetry.addLine("========================================");
                telemetry.addLine("Current PIDF Constants:");
                telemetry.addData("", "SHOOTER_P = %.3f", shooter_kP);
                telemetry.addData("", "SHOOTER_I = %.3f", shooter_kI);
                telemetry.addData("", "SHOOTER_D = %.3f", shooter_kD);
                telemetry.addData("", "SHOOTER_F = %.3f", shooter_kF);
                    break;

                case 2: // Turret
                    telemetry.addLine("🔧 TURRET PID TUNING MODE ACTIVE 🔧");
                    telemetry.addLine("========================================");

                    // Calculate turret errors
                    double turretTarget = shooter.getTargetTurretAngle();
                    double turretPosition = shooter.getTurretPosition();
                    double turretError = turretTarget - turretPosition;

                    // Turret Information
                    telemetry.addLine("--- TURRET POSITION ---");
                    telemetry.addData("Target Position", "%.2f° servo", turretTarget);
                    telemetry.addData("Current Position", "%.2f° servo", turretPosition);
                    telemetry.addData("Position Error", "%.2f°", turretError);
                    telemetry.addLine("");

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
                    telemetry.addLine("Y Button: Cycle subsystem");
                    telemetry.addLine("Back Button: Exit tuning mode");
                    telemetry.addLine("========================================");
                    telemetry.addLine("Current PID Constants:");
                    telemetry.addData("", "TURRET_P = %.5f", turret_kP);
                    telemetry.addData("", "TURRET_I = %.5f", turret_kI);
                    telemetry.addData("", "TURRET_D = %.5f", turret_kD);
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
                telemetry.addLine("Y Button: Cycle subsystem | Right Stick: Jump to Turret");
                telemetry.addLine("Back Button: Exit tuning mode");
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