package teleOps;

import Constants.EnumConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.ShooterConstants;
import Constants.ShootingSequenceConstants;
import Constants.SpindexerConstants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotHardware;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.Intake;
import subsystems.Spindexer;
import subsystems.Limelight;
import subsystems.Turret;
import subsystems.Odometry;
import commands.CatalogCommands;
import utility.SpindexerAndMotifStatus;

/**
 * Standalone Red alliance demo TeleOp with restricted functionality.
 *
 * Disabled: Flywheel, driving, shooting command sequence.
 * Active: Intake, cataloging, spindexer, limelight, turret tracking, manual hood control.
 * Right joystick Y controls hood angle directly.
 * Right trigger does a timed shooter flipper flick (no flywheel spinup).
 */
@Disabled
@TeleOp
public class DemoTeleOp extends CommandOpMode {

    private final RobotHardware robot = RobotHardware.getInstance();
    private GamepadEx driverGamepad;

    // Subsystems (NO Shooter, NO MecanumDrive)
    private Intake intake;
    private Spindexer spindexer;
    private Limelight limelight;
    private Turret turret;
    private Odometry odometry;

    // Shooter flipper state machine (direct servo control, no Shooter subsystem)
    private EnumConstants.FlickState shooterFlickState = EnumConstants.FlickState.Idle;
    private final ElapsedTime shooterFlickTimer = new ElapsedTime();

    private boolean servosInitialized = false;

    @Override
    public void initialize() {
        // Set alliance to Red
        RobotConstants.Robot.allianceColor = EnumConstants.AllianceColor.Red;

        // Initialize hardware
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        // Set pinpoint position for turret tracking
        com.pedropathing.geometry.Pose startPose;
        if (OdometryConstants.endingAutonPose != null) {
            startPose = OdometryConstants.endingAutonPose;
        } else {
            startPose = OdometryConstants.redStartPoint;
        }
        robot.pinpoint.setPosition(OdometryConstants.toPose2D(startPose));
        robot.pinpoint.update();

        // Set default shooting mode
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;

        // Create only the subsystems we need (NO Shooter, NO MecanumDrive)
        intake = new Intake();
        spindexer = new Spindexer();
        limelight = new Limelight();
        turret = new Turret();
        odometry = new Odometry();

        // Register subsystems with CommandScheduler so periodic() is called
        register(intake, spindexer, limelight, turret, odometry);

        // SAFETY: Force shooter motors to zero - they must NEVER spin
        robot.shooterMotor1.setPower(0);
        robot.shooterMotor2.setPower(0);

        // Configure button bindings
        configureButtonBindings();

        // Start Limelight in TagTracking mode
        limelight.setMode(EnumConstants.LimelightMode.TagTracking);
    }

    private void configureButtonBindings() {
        // NO drivetrain default command - robot cannot drive

        // Intake controls
        new Trigger(() -> gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> intake.stopIntake());

        // Right trigger: Timed shooter flipper flick (NO flywheel, NO sequence)
        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> triggerShooterFlick());

        // A button: Toggle shooting mode (Fast/Sorted)
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    SpindexerConstants.currentMode = (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Fast)
                            ? EnumConstants.ShootingMode.Sorted
                            : EnumConstants.ShootingMode.Fast;
                });

        // X button: Manual spindexer flick
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(spindexer::triggerFlick));

        // Y button: Catalog balls (mode-dependent)
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
                        EnumConstants.BallColor[] motifPattern = {
                                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0),
                                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),
                                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)
                        };
                        EnumConstants.BallColor[] intakeColors = {
                                robot.intakeSensorPair.quickCheck().color,
                                robot.transferSensorPair.quickCheck().color,
                                robot.rampSensorPair.quickCheck().color
                        };
                        schedule(CatalogCommands.catalogSorted(spindexer, intake, motifPattern, intakeColors));
                    } else {
                        schedule(CatalogCommands.catalogFast(spindexer, intake));
                    }
                });

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(spindexer::rotateCCW));
        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(spindexer::rotateCW));

        // Limelight controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(limelight::toggleMode));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(limelight::resetLimelight));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(intake::reverse))
                .whenReleased(new InstantCommand(intake::stopIntake));
    }

    @Override
    public void run() {
        if (!servosInitialized) {
            spindexer.initServoPositions();
            turret.initServoPositions();
            robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
            setHoodAngleDirect(ShooterConstants.HOOD_DEFAULT_ANGLE);
            servosInitialized = true;
        }

        super.run();

        // Update odometry every loop (required for turret tracking)
        robot.pinpoint.update();

        // SAFETY: Force shooter motors to zero EVERY loop
        robot.shooterMotor1.setPower(0);
        robot.shooterMotor2.setPower(0);

        // Hood angle control from right joystick
        updateHoodFromJoystick();

        // Shooter flipper state machine
        shooterFlipperStateMachine();

    }

    /**
     * Maps right joystick Y to hood angle within safety limits.
     * Stick up (-1) = 30.1 deg (more vertical), stick down (+1) = 62.9 deg (more horizontal).
     */
    private void updateHoodFromJoystick() {
        double stickY = gamepad1.right_stick_y; // -1 = up, +1 = down

        double safetyMin = ShooterConstants.HOOD_MIN_ANGLE + 3; // 33
        double safetyMax = ShooterConstants.HOOD_MAX_ANGLE - 3; // 60

        // Map stick [-1, +1] to [0, 1]
        double normalized = (stickY + 1.0) / 2.0;

        // Map to angle range: 0 -> safetyMin, 1 -> safetyMax
        double hoodAngle = safetyMin + normalized * (safetyMax - safetyMin);

        setHoodAngleDirect(hoodAngle);
    }

    /**
     * Sets the hood servo position directly, bypassing the Shooter subsystem.
     * Replicates the angle-to-servo conversion from Shooter.hoodAngleToServoPosition().
     */
    private void setHoodAngleDirect(double angleDegrees) {
        // Clamp to valid range
        angleDegrees = Math.max(ShooterConstants.HOOD_MIN_ANGLE,
                Math.min(ShooterConstants.HOOD_MAX_ANGLE, angleDegrees));

        // Linear mapping: 30 deg -> servo 0.88, 63 deg -> servo 0.26
        double angleRange = ShooterConstants.HOOD_MAX_ANGLE - ShooterConstants.HOOD_MIN_ANGLE;
        double servoRange = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE;
        double normalizedAngle = (angleDegrees - ShooterConstants.HOOD_MIN_ANGLE) / angleRange;
        double position = ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE - (normalizedAngle * servoRange);

        // Clamp to valid servo range
        position = Math.max(ShooterConstants.HOOD_SERVO_AT_MAX_ANGLE,
                Math.min(ShooterConstants.HOOD_SERVO_AT_MIN_ANGLE, position));

        robot.shooterHood.setPosition(position);
    }

    /**
     * Triggers the shooter flipper flick if idle.
     */
    private void triggerShooterFlick() {
        if (shooterFlickState == EnumConstants.FlickState.Idle) {
            shooterFlickState = EnumConstants.FlickState.Start;
        }
    }

    /**
     * Runs the shooter flipper state machine each loop.
     * Directly controls robot.shooterFlipper servo without the Shooter subsystem.
     * Matches the timing from Shooter.flipperStateMachinePeriodic().
     */
    private void shooterFlipperStateMachine() {
        if (shooterFlickState == EnumConstants.FlickState.Idle) return;

        switch (shooterFlickState) {
            case Start:
                robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_EXTENDED);
                shooterFlickTimer.reset();
                shooterFlickState = EnumConstants.FlickState.Extended;
                break;
            case Extended:
                if (shooterFlickTimer.seconds() < ShootingSequenceConstants.SHOOTER_FLICK_TIME) break;
                robot.shooterFlipper.setPosition(ShootingSequenceConstants.SHOOTER_FLIPPER_RETRACT);
                shooterFlickTimer.reset();
                shooterFlickState = EnumConstants.FlickState.Retracted;
                break;
            case Retracted:
                shooterFlickState = EnumConstants.FlickState.Idle;
                break;
        }
    }
}
