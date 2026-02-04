package teleOps;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import subsystems.*;
import utility.*;
import commands.ShootingCommands;
import commands.CatalogCommands;
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

    private ShootingValidator shootingValidator;
    private TelemetryHelper telemetryHelper;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private SimplePoseTracker poseTracker;
    private double loopMs;




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

        // Link Odometry to Shooter for field state
        shooter.setOdometry(odometry);

        shootingValidator = new ShootingValidator(odometry, telemetry);

        telemetryHelper = new TelemetryHelper();
        telemetryHelper.setSubsystems(shooter, turret, spindexer, odometry, limelight,
                mecanumDrive, intake);
        telemetryHelper.setGamepad(gamepad1);

        poseTracker = new SimplePoseTracker();
        FieldDrawing.init();

        register(mecanumDrive, intake, shooter, spindexer, limelight, turret, odometry);
    }

    protected void configureButtonBindings() {
        mecanumDrive.setDefaultCommand(
                new RunCommand(() -> {
                    double[] controls = getTransformedControls();
                    mecanumDrive.drive(controls[0], controls[1], controls[2]);
                }, mecanumDrive)
        );

        new Trigger(() -> gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> intake.stopIntake());

        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> {
                    boolean override = gamepad1.right_stick_button;
                    if (!shootingValidator.canShoot(override)) {
                        gamepad1.rumble(200);
                        return;
                    }
                    schedule(ShootingCommands.shootThreeBalls(shooter, spindexer));
                });

        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(mecanumDrive::resetYaw));
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(mecanumDrive::toggleSlowMode));

        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    SpindexerConstants.currentMode = (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Fast)
                            ? EnumConstants.ShootingMode.Sorted
                            : EnumConstants.ShootingMode.Fast;
                });

        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(spindexer::triggerFlick));

        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
                        EnumConstants.BallColor[] motifPattern = {SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0), SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)};
                        EnumConstants.BallColor[] intakeColors = {robot.intakeSensorPair.quickCheck().color,robot.transferSensorPair.quickCheck().color,robot.rampSensorPair.quickCheck().color};
                        schedule(CatalogCommands.catalogSorted(spindexer, intake, motifPattern, intakeColors));
                    } else {
                        schedule(CatalogCommands.catalogFast(spindexer, intake));
                    }
                });

        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCCW));

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCW));

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(limelight::toggleMode));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(limelight::resetLimelight));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(intake::reverse))
                .whenReleased(new InstantCommand(intake::stopIntake));

        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new RelocalizePinpointCommand(limelight));
    }

    @Override
    public void run() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();

        robot.pinpoint.update();

        super.run();

        double poseX = robot.pinpoint.getPosX(DistanceUnit.INCH);
        double poseY = robot.pinpoint.getPosY(DistanceUnit.INCH);
        poseTracker.addPose(poseX, poseY);

        limelight.updateLimelightPose();

        telemetry.addData("Loop", "%.1f ms (%.0f Hz)", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0);
        updateTelemetry();
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
        double rawY = -gamepad1.left_stick_y;
        double rawX = gamepad1.left_stick_x;
        double rawRotation = gamepad1.right_stick_x;

        EnumConstants.AllianceColor invertedAlliance = RobotConstants.Controls.SWAP_ALLIANCE_CONTROLS
                ? EnumConstants.AllianceColor.Blue
                : EnumConstants.AllianceColor.Red;

        if (RobotConstants.Robot.allianceColor != null &&
                RobotConstants.Robot.allianceColor == invertedAlliance) {
            return new double[] {-rawY, -rawX, rawRotation};
        } else {
            return new double[] {rawY, rawX, rawRotation};
        }
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
        telemetry.addData("bruh:", loopMs);

        if (RobotConstants.Robot.ENABLE_TELEMETRY) {
            double poseX = robot.pinpoint.getPosX(DistanceUnit.INCH);
            double poseY = robot.pinpoint.getPosY(DistanceUnit.INCH);
            double poseH = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            Pose currentPose = new Pose(poseX, poseY, poseH);
            FieldDrawing.drawTeleOpDebug(
                    poseTracker.getXArray(),
                    poseTracker.getYArray(),
                    poseTracker.getCount(),
                    currentPose);
        }
    }
}