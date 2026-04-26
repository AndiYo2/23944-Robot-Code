package teleOps;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import Constants.TurretConstants;

import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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

import java.util.function.Supplier;

import pedroPathing.Constants;
import subsystems.*;
import utility.*;
import commands.ShootingCommands;
import commands.CatalogCommands;
import commands.ParkCommand;
import commands.RelocalizePinpointCommand;
import commands.AggressiveReturnCommand;
import com.arcrobotics.ftclib.command.Command;
import Constants.EnumConstants.DriveState;

abstract public class TeleOpTemplate extends CommandOpMode {
    protected MecanumDrive mecanumDrive;
    protected Shooter shooter;
    protected Turret turret;
    protected Intake intake;
    protected Spindexer spindexer;
    protected subsystems.Limelight limelight;
    protected GamepadEx mainController;
    protected GamepadEx secondaryController;
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
    private boolean endGame = false;

    // Pedro in Teleop
    private Follower follower;
    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> intakePathChain;
    private boolean autoDrive = false;

    // Defense anchor (LT): snapshot pose on press, while held X-Lock when close
    // to anchor, otherwise follow an aggressive return path back to it.
    private static final double DEFENSE_AT_ANCHOR_DIST = 0.3;
    private static final double DEFENSE_BRAKE_START = 0.15;
    private static final double DEFENSE_BRAKE_STRENGTH = 2.0;
    private static final double DEFENSE_RETURN_MAX_POWER = 1.0;
    private static final double LT_ENGAGE_THRESHOLD = 0.5;
    private static final double LT_RELEASE_THRESHOLD = 0.15;
    private Pose anchorPose;
    private boolean ltEngaged = false;
    private Command activeReturnCommand;


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
        follower = Constants.createFollower(hardwareMap);
        Pose gatePose = (allianceColor == EnumConstants.AllianceColor.Red)
                ? OdometryConstants.redGatePose : OdometryConstants.blueGatePose;
        Pose intakePose = (allianceColor == EnumConstants.AllianceColor.Red)
                ? OdometryConstants.redIntakePose : OdometryConstants.blueIntakePose;
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, gatePose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, gatePose.getHeading(), 0.8))
                .build();
        intakePathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, intakePose)))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, intakePose.getHeading(), 0.8))
                .build();
        initHardware();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (OdometryConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(OdometryConstants.toPose2D(fallbackStartPosition));
            robot.pinpoint.update();
            follower.update();
        }

        follower.startTeleopDrive();

        configureButtonBindings();

        telemetry.addData("Position After Set", follower.getPose());
        telemetry.addData("Pinpoint", robot.pinpoint.getPosition());
        telemetry.update();

        limelight.setMode(EnumConstants.LimelightMode.GoalTracking);
    }

    protected void initHardware() {
        mainController = new GamepadEx(gamepad1);
        secondaryController = new GamepadEx(gamepad2);
        robot.init(hardwareMap, mainController, false);

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
        spindexer = new Spindexer();
        limelight = new subsystems.Limelight();

        shooter.setTurret(turret);

        turret.setShooter(shooter);

        telemetryHelper = new TelemetryHelper();
        telemetryHelper.setSubsystems(shooter, turret, spindexer, limelight,
                mecanumDrive, intake);

        poseTracker = new SimplePoseTracker();
        FieldDrawing.init();

        register(mecanumDrive, intake, shooter, spindexer, limelight, turret);
    }

    protected void configureButtonBindings() {
        mecanumDrive.setDefaultCommand(
                new RunCommand(() -> {
                    if (!autoDrive) {
                        double[] controls = getTransformedControls();
                        mecanumDrive.drive(controls[0], controls[1], controls[2]);
                    }
                }, mecanumDrive)
        );

        // ============ MAIN CONTROLLER (gamepad1) ============

        // Dpad Up — intake (hold)
        new GamepadButton(mainController, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(intake::runIntake))
                .whenReleased(new InstantCommand(intake::stopIntake));

        // Dpad Down — catalogue
        new GamepadButton(mainController, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> schedule(CatalogCommands.catalogFast(spindexer, intake, robot.spindexerSensorPair)));

        // Left bumper — rotate CCW spindexer
        new GamepadButton(mainController, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCCW));

        // Right bumper — rotate CW spindexer
        new GamepadButton(mainController, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCW));

        // Left trigger — defense anchor. Handled per-loop in run() so the
        // Schmitt trigger / distance check / path rebuild can all run continuously.

        // Right trigger — shoot
        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> schedule(!endGame ?
                        ShootingCommands.shootThreeBalls(shooter, spindexer) :
                        ShootingCommands.superSlowShootThreeBalls(shooter, spindexer)));

        // Triangle/Y — relocalize
        new GamepadButton(mainController, GamepadKeys.Button.Y)
                .whenPressed(new RelocalizePinpointCommand(limelight));

        // Square/X — spindexer flipper
        new GamepadButton(mainController, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(spindexer::triggerFlick));

        // X/A (down) — toggle slow shooting mode
        new GamepadButton(mainController, GamepadKeys.Button.A)
                .whenPressed(() -> endGame = !endGame);

        // Circle/B — toggle slow driving mode
        new GamepadButton(mainController, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(mecanumDrive::toggleSlowMode));

        // Options/Start — IMU reset
        new GamepadButton(mainController, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(mecanumDrive::resetYaw));

        // ============ SECONDARY CONTROLLER (gamepad2) ============

        // Dpad Up — increase velocity offset by 20
        new GamepadButton(secondaryController, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> Shooter.runtimeVelocityOffset += 20));

        // Dpad Down — decrease velocity offset by 20
        new GamepadButton(secondaryController, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> Shooter.runtimeVelocityOffset -= 20));

        // Left bumper — aim turret left (increase tracking offset)
        new GamepadButton(secondaryController, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    if (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                        TurretConstants.BLUE_TURRET_TRACKING_OFFSET += 1;
                    else
                    {
                        TurretConstants.RED_FRONT_TURRET_TRACKING_OFFSET += 1;
                        TurretConstants.RED_BACK_TURRET_TRACKING_OFFSET += 1;
                    }
                }));

        // Right bumper — aim turret right (decrease tracking offset)
        new GamepadButton(secondaryController, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(() -> {
                    if (RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue)
                        TurretConstants.BLUE_TURRET_TRACKING_OFFSET -= 1;
                    else
                    {
                        TurretConstants.RED_FRONT_TURRET_TRACKING_OFFSET -= 1;
                        TurretConstants.RED_BACK_TURRET_TRACKING_OFFSET -= 1;
                    }
                }));

        // Options/Start — park
        new GamepadButton(secondaryController, GamepadKeys.Button.START)
                .whenPressed(new ParkCommand());

        // X/A (down) — toggle moving while shooting
        new GamepadButton(secondaryController, GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(() ->
                        ShooterConstants.SHOOTING_WHILE_MOVING_ENABLED =
                                !ShooterConstants.SHOOTING_WHILE_MOVING_ENABLED));

        // Circle/B — toggle manual override (turret center, velocity 2000, hood 53)
        new GamepadButton(secondaryController, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() ->
                        ShooterConstants.MANUAL_OVERRIDE = !ShooterConstants.MANUAL_OVERRIDE));

        // Triangle/Y — reset all manual offsets to defaults
        new GamepadButton(secondaryController, GamepadKeys.Button.Y)
                .whenPressed(new InstantCommand(() -> {
                    Shooter.runtimeVelocityOffset = 0;
                    TurretConstants.BLUE_TURRET_TRACKING_OFFSET = 0;
                    TurretConstants.RED_FRONT_TURRET_TRACKING_OFFSET = 3.0;
                    TurretConstants.RED_BACK_TURRET_TRACKING_OFFSET = 3.0;
                }));
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
        telemetryHelper.recordLoop(loopMs);

        robot.clearBulkCache();

        follower.update();
        robot.updateCachedPose();

        updateDefenseAnchor();

        super.run();

        poseTracker.addPose(robot.cachedPoseX, robot.cachedPoseY);

        if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
            telemetryTimer.reset();
            telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
            updateTelemetry();
        }
    }
    /**
     * Per-loop defense-anchor logic bound to LEFT_TRIGGER.
     * On rising edge: snapshot current pose. While held: if within
     * DEFENSE_AT_ANCHOR_DIST → X-Lock; otherwise schedule an AggressiveReturnCommand
     * at max power back to the anchor. On falling edge: cancel + resume teleop drive.
     */
    private void updateDefenseAnchor() {
        double lt = gamepad1.left_trigger;
        boolean wasEngaged = ltEngaged;
        if (!ltEngaged && lt > LT_ENGAGE_THRESHOLD) ltEngaged = true;
        else if (ltEngaged && lt < LT_RELEASE_THRESHOLD) ltEngaged = false;

        // Rising edge: snapshot anchor pose.
        if (ltEngaged && !wasEngaged) {
            anchorPose = follower.getPose();
        }

        // Falling edge: cancel return, drop X-Lock, resume teleop driving.
        if (!ltEngaged && wasEngaged) {
            if (activeReturnCommand != null) {
                if (!activeReturnCommand.isFinished()) activeReturnCommand.cancel();
                activeReturnCommand = null;
            }
            if (mecanumDrive.getCurrentState() == DriveState.Locked) {
                mecanumDrive.setXLock(false);
            }
            if (mecanumDrive.getCurrentState() == DriveState.AutoDriving) {
                mecanumDrive.setAutoDriving(false);
            }
            follower.breakFollowing();
            follower.startTeleopDrive();
            autoDrive = false;
            return;
        }

        if (!ltEngaged || anchorPose == null) return;

        Pose current = follower.getPose();
        double dx = current.getX() - anchorPose.getX();
        double dy = current.getY() - anchorPose.getY();
        double distance = Math.hypot(dx, dy);

        if (distance < DEFENSE_AT_ANCHOR_DIST) {
            // Close → cancel any running path, engage X-Lock, let drive() write the pattern.
            if (activeReturnCommand != null) {
                if (!activeReturnCommand.isFinished()) activeReturnCommand.cancel();
                activeReturnCommand = null;
            }
            if (mecanumDrive.getCurrentState() == DriveState.AutoDriving) {
                mecanumDrive.setAutoDriving(false);
            }
            autoDrive = false;
            if (mecanumDrive.getCurrentState() != DriveState.Locked) {
                mecanumDrive.setXLock(true);
            }
        } else {
            // Away → release X-Lock, ensure an aggressive return path is running.
            if (mecanumDrive.getCurrentState() == DriveState.Locked) {
                mecanumDrive.setXLock(false);
            }
            if (activeReturnCommand == null || activeReturnCommand.isFinished()) {
                activeReturnCommand = new AggressiveReturnCommand(
                        follower, anchorPose, DEFENSE_RETURN_MAX_POWER,
                        DEFENSE_BRAKE_START, DEFENSE_BRAKE_STRENGTH);
                mecanumDrive.setAutoDriving(true);
                autoDrive = true;
                schedule(activeReturnCommand);
            }
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
            FieldDrawing.drawRobot(
                    limelight.getLimelightPose(),
                    new Style("", "#FFFFFF", 0.75)
            );
            FieldDrawing.drawTeleOpDebug(
                    poseTracker.getXArray(),
                    poseTracker.getYArray(),
                    poseTracker.getCount(),
                    currentPose);
        }
    }
}
