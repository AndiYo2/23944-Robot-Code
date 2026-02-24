package teleOps;

import Constants.DriveConstants;
import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import subsystems.*;
import utility.*;
import commands.ShootingCommands;
import commands.CatalogCommands;
import commands.ParkCommand;
import commands.RelocalizePinpointCommand;

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Turret turret;
    protected Odometry odometry;
    protected Intake intake;
    protected Spindexer spindexer;
    protected subsystems.Limelight limelight;
    protected GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();

    private TelemetryHelper telemetryHelper;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 200; // ~5 Hz
    private SimplePoseTracker poseTracker;
    private double loopMs;

    private boolean servosInitialized = false;

    // Pre-allocated array for getTransformedControls() to avoid GC pressure
    private final double[] controlsArray = new double[3];


    /**
     * Consolidated initialization for alliance-specific TeleOp.
     * Call this from subclasses with the alliance color and fallback start position.
     *
     * @param allianceColor the alliance color (Blue or Red)
     * @param fallbackStartPosition the start position to use if no auton ran
     */
    protected void initForAlliance(EnumConstants.AllianceColor allianceColor,
                                   com.pedropathing.geometry.Pose fallbackStartPosition) {
        // Set alliance color BEFORE initHardware so it can use the correct settings
        RobotConstants.Robot.allianceColor = allianceColor;
        initHardware();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (OdometryConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(OdometryConstants.toPose2D(fallbackStartPosition));
            robot.pinpoint.update();
        }

        configureButtonBindings();

        limelight.setMode(EnumConstants.LimelightMode.TagTracking);
    }

    protected void initHardware() {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        // Set starting position for TeleOp
        // If we have an ending auton pose, use it; otherwise use standard start point
        com.pedropathing.geometry.Pose startPose;
        if (OdometryConstants.endingAutonPose != null) {
            startPose = OdometryConstants.endingAutonPose;
            telemetry.addData("TeleOp Init", "Using Auton End Position");
        } else {
            startPose = OdometryConstants.standardStartPoint;
            telemetry.addData("TeleOp Init", "Using Standard Start Position");
        }

        telemetry.addData("Setting Position To", startPose);
        telemetry.update();

        robot.pinpoint.setPosition(OdometryConstants.toPose2D(startPose));
        robot.pinpoint.update();

        telemetry.addData("Position After Set", robot.pinpoint.getPosition());
        telemetry.update();

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive();
        intake = new Intake();
        shooter = new Shooter();
        turret = new Turret();
        odometry = new Odometry();
        spindexer = new Spindexer();
        limelight = new subsystems.Limelight();

        // Link Turret to Shooter for distance calculations
        shooter.setTurret(turret);

        // Link Shooter to Turret for lead-compensated aiming
        turret.setShooter(shooter);

        telemetryHelper = new TelemetryHelper();
        telemetryHelper.setSubsystems(shooter, turret, spindexer, odometry, limelight,
                mecanumDrive, intake);

        poseTracker = new SimplePoseTracker();
        FieldDrawing.init();

        register(mecanumDrive, intake, shooter, spindexer, limelight, turret, odometry);
    }

    protected void configureButtonBindings() {
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

        // Right trigger — shoot
        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> {
                    schedule(ShootingCommands.shootThreeBalls(shooter, spindexer));
                });

        // Left trigger — dynamic slow mode (handled in drive default command)

        // Left bumper — rotate CCW spindexer
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCCW));

        // Dpad Up — reset limelight
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(limelight::resetLimelight));
        // Dpad Down — rotate CW spindexer
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(this::manualRotateCW));
        // Dpad Left — relocalize
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new RelocalizePinpointCommand(limelight));
        // Dpad Right — invert intake (hold)
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(intake::reverse))
                .whenReleased(new InstantCommand(intake::stopIntake));

        // Triangle/Y — catalogue
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
                        EnumConstants.BallColor[] motifPattern = {SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0), SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)};
                        // Order matches physical ball path: spindexer slot (first in) → transfer → ramp (last in)
                        EnumConstants.BallColor[] intakeColors = {robot.intakeSensorPair.quickCheck().color,robot.transferSensorPair.quickCheck().color,robot.rampSensorPair.quickCheck().color};
                        schedule(CatalogCommands.catalogSorted(spindexer, intake, motifPattern, intakeColors));
                    } else {
                        schedule(CatalogCommands.catalogFast(spindexer, intake));
                    }
                });

        // X/A — toggle sorting mode
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    SpindexerConstants.currentMode = (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Fast)
                            ? EnumConstants.ShootingMode.Sorted
                            : EnumConstants.ShootingMode.Fast;
                });

        // Square/X — manual spindexer flipper
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(spindexer::triggerFlick));

        // Circle/B — toggle slow mode
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(mecanumDrive::toggleSlowMode));

        // Options/Start — reset IMU/yaw
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(mecanumDrive::resetYaw));

        // Share/Back — deploy park mechanism
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)
                .whenPressed(new ParkCommand());
    }

    @Override
    public void run() {
        if (!servosInitialized) {
            shooter.initServoPositions();
            turret.initServoPositions();
            spindexer.initServoPositions();
            servosInitialized = true;
        }

        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        robot.clearBulkCache();
        robot.pinpoint.update();
        robot.updateCachedPose();
        robot.pollNextSensor();

        ShooterConstants.SHOOT_WHILE_MOVING_ENABLED = false;

        super.run();

        poseTracker.addPose(robot.cachedPoseX, robot.cachedPoseY);

        if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
            telemetryTimer.reset();
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
            updateTelemetry();
        }
    }

    /**
     * Applies alliance-specific control mapping transformations.
     * Blue alliance uses standard field coordinates.
     * Red alliance inverts X and Y axes to account for mirrored starting position.
     * Use SWAP_ALLIANCE_CONTROLS in RobotConstants to swap which alliance gets inverted controls.
     *
     * @return double array [fieldY, fieldX, rotation] — pre-allocated, do NOT store reference
     */
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

    /**
     * Computes a speed multiplier from the left trigger for dynamic slow mode.
     * No press (below deadband) = 1.0 (full speed), full press = DYNAMIC_SLOW_MIN (0.25).
     */
    private double getDynamicSlowMultiplier() {
        double trigger = gamepad1.left_trigger;
        if (trigger < DriveConstants.DYNAMIC_SLOW_DEADBAND) {
            return 1.0;
        }
        double normalized = (trigger - DriveConstants.DYNAMIC_SLOW_DEADBAND)
                / (1.0 - DriveConstants.DYNAMIC_SLOW_DEADBAND);
        return 1.0 - normalized * (1.0 - DriveConstants.DYNAMIC_SLOW_MIN);
    }

    /**
     * Manual rotation forward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     * OVERRIDE: Hold left stick button to force rotation during sequence
     */
    private void manualRotateCW() {
        spindexer.rotateCW();
    }

    /**
     * Manual rotation backward with ball pattern update
     * Blocked during shooting sequence to prevent conflicts
     * OVERRIDE: Hold left stick button to force rotation during sequence
     */
    private void manualRotateCCW() {
        spindexer.rotateCCW();
    }


    private void updateTelemetry() {
        telemetryHelper.update(telemetry, loopMs);

        if (RobotConstants.Robot.ENABLE_TELEMETRY) {
            Pose currentPose = new Pose(robot.cachedPoseX, robot.cachedPoseY, robot.cachedHeading);
            FieldDrawing.drawTeleOpDebug(
                    poseTracker.getXArray(),
                    poseTracker.getYArray(),
                    poseTracker.getCount(),
                    currentPose);
        }
    }
}
