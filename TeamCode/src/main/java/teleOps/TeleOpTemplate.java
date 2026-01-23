package teleOps;

import Constants.EnumConstants;
import Constants.FieldMap;
import Constants.LimelightConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import Constants.ShooterConstants;
import com.qualcomm.robotcore.hardware.DcMotor;
import utility.RobotHardware;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import subsystems.*;
import utility.*;
import utility.ShootingValidator;
import commands.ShootingCommands;
import commands.CatalogCommands;
import commands.ScanSensorsCommand;

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

    // Ball tracking (replaces SpindexerManager state)
    private int totalBallsInRobot = 0;
    private EnumConstants.BallColor ball1Color = EnumConstants.BallColor.None;
    private EnumConstants.BallColor ball2Color = EnumConstants.BallColor.None;
    private EnumConstants.BallColor ball3Color = EnumConstants.BallColor.None;
    private EnumConstants.ShootingMode currentMode = EnumConstants.ShootingMode.Fast;




    /**
     * Consolidated initialization for alliance-specific TeleOp.
     * Call this from subclasses with the alliance color and fallback start position.
     *
     * @param allianceColor the alliance color (Blue or Red)
     * @param fallbackStartPosition the start position to use if no auton ran
     */
    protected void initForAlliance(EnumConstants.AllianceColor allianceColor,
                                   org.firstinspires.ftc.robotcore.external.navigation.Pose2D fallbackStartPosition) {
        // Set alliance color BEFORE initHardware so it can use the correct settings
        RobotConstants.Robot.allianceColor = allianceColor;
        initHardware(false);

        // Only set position if no auton ran (endingAutonPose is null)
        // If auton ran, initHardware already set the position from endingAutonPose
        if (OdometryConstants.endingAutonPose == null) {
            robot.pinpoint.setPosition(fallbackStartPosition);
            robot.pinpoint.update();
        }

        configureButtonBindings();
    }

    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        // Set starting position for TeleOp
        // If we have an ending auton pose, use it; otherwise use standard start point
        org.firstinspires.ftc.robotcore.external.navigation.Pose2D startPosition;
        if (OdometryConstants.endingAutonPose != null) {
            // Convert Pedro Pose to FTC Pose2D
            com.pedropathing.geometry.Pose autonPose = OdometryConstants.endingAutonPose;
            startPosition = new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(
                    org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                    autonPose.getX(),
                    autonPose.getY(),
                    org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS,
                    autonPose.getHeading());
            telemetry.addData("TeleOp Init", "Using Auton End Position");
        } else {
            // No auton ran - use standard starting position
            startPosition = OdometryConstants.standardStartPoint;
            telemetry.addData("TeleOp Init", "Using Standard Start Position");
        }

        telemetry.addData("Setting Position To", startPosition);
        telemetry.update();

        robot.pinpoint.setPosition(startPosition);
        // CRITICAL: Update Pinpoint after setting position to apply it
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

        // Link Odometry to Shooter for field state
        shooter.setOdometry(odometry);

        // Set turret default command to track goal based on field state
        turret.setDefaultCommand(new RunCommand(() -> {
            turret.turretPeriodic(odometry.getFieldState());
        }, turret));

        shootingValidator = new ShootingValidator(odometry, telemetry);

        register(mecanumDrive, intake, shooter, spindexer, limelight, turret, odometry);
    }

    protected void configureButtonBindings() {
        // Default command for drivetrain - runs every loop when no other command requires it
        mecanumDrive.setDefaultCommand(
                new RunCommand(() -> {
                    double[] controls = getTransformedControls();
                    mecanumDrive.drive(controls[0], controls[1], controls[2]);
                }, mecanumDrive)
        );

        // Intake controls
        new Trigger(() -> gamepad1.left_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> intake.runIntake())
                .whenInactive(() -> intake.stopIntake());

        // Shooting controls (with zone validation)
        // Right trigger: shoots all balls with validation
        new Trigger(() -> gamepad1.right_trigger > RobotConstants.Controls.TRIGGER_THRESHOLD)
                .whenActive(() -> {
                    boolean override = gamepad1.right_stick_button;
                    if (!shootingValidator.canShoot(override)) {
                        gamepad1.rumble(200);
                        return;
                    }
                    schedule(ShootingCommands.shootAllBalls(shooter, spindexer, totalBallsInRobot));
                });

        // Drive controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(mecanumDrive::resetYaw));
        new GamepadButton(driverGamepad, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(mecanumDrive::toggleSlowMode));

        // A button: Toggle shooting mode
        new GamepadButton(driverGamepad, GamepadKeys.Button.A)
                .whenPressed(() -> {
                    currentMode = (currentMode == EnumConstants.ShootingMode.Fast)
                            ? EnumConstants.ShootingMode.Sorted
                            : EnumConstants.ShootingMode.Fast;
                });

        // X button: Manual flick
        new GamepadButton(driverGamepad, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(spindexer::triggerFlick));

        // Y button: Catalog balls
        new GamepadButton(driverGamepad, GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    scanAndCountBalls();
                    schedule(CatalogCommands.catalogFast(spindexer, shooter, intake,
                            robot.intakeSensorPair, robot.transferSensorPair, robot.rampSensorPair,
                            this::updateBallTracking));
                });

        // Manual spindexer controls
        new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCCW));

        new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new InstantCommand(this::manualRotateCW));

        // Mode toggles
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(limelight::toggleMode));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(limelight::resetLimelight));
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(intake::reverse));
    }

    @Override
    public void run() {
        super.run();

        // CRITICAL: Update Pinpoint odometry every loop (like in test OpMode)
        robot.pinpoint.update();

        updateSubsystems();
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
        double rawY = -gamepad1.left_stick_y;  // Negative because gamepad Y is inverted
        double rawX = gamepad1.left_stick_x;
        double rawRotation = gamepad1.right_stick_x;

        // Determine which alliance should have inverted controls
        EnumConstants.AllianceColor invertedAlliance = RobotConstants.Controls.SWAP_ALLIANCE_CONTROLS
                ? EnumConstants.AllianceColor.Blue
                : EnumConstants.AllianceColor.Red;

        // Check alliance color (defaults to Blue if not set)
        if (RobotConstants.Robot.allianceColor != null &&
                RobotConstants.Robot.allianceColor == invertedAlliance) {
            // Invert both translational axes (180° field rotation)
            return new double[] {-rawY, -rawX, rawRotation};
        } else {
            // Standard mapping
            return new double[] {rawY, rawX, rawRotation};
        }
    }

    /**
     * Scans all three sensors and counts total balls in robot.
     * Updates ball colors and totalBallsInRobot count.
     */
    private void scanAndCountBalls() {
        DualBallDetector.Result r1 = robot.intakeSensorPair.detectBall();
        DualBallDetector.Result r2 = robot.transferSensorPair.detectBall();
        DualBallDetector.Result r3 = robot.rampSensorPair.detectBall();

        ball1Color = r1.ballPresent ? r1.color : EnumConstants.BallColor.None;
        ball2Color = r2.ballPresent ? r2.color : EnumConstants.BallColor.None;
        ball3Color = r3.ballPresent ? r3.color : EnumConstants.BallColor.None;

        // If ramp has a ball but transfer doesn't, assume transfer has purple
        if (r3.ballPresent && !r2.ballPresent) {
            ball2Color = EnumConstants.BallColor.Purple;
        }

        // Count total balls from all sensors
        totalBallsInRobot = 0;
        if (r1.ballPresent) totalBallsInRobot++;
        if (r2.ballPresent || (r3.ballPresent && !r2.ballPresent)) totalBallsInRobot++;
        if (r3.ballPresent) totalBallsInRobot++;
    }

    /**
     * Callback for CatalogCommands to update ball tracking after scan.
     */
    private void updateBallTracking(ScanSensorsCommand.ScanResults results) {
        ball1Color = results.sensor0.ballPresent ? results.sensor0.color : EnumConstants.BallColor.None;
        ball2Color = results.sensor1.ballPresent ? results.sensor1.color : EnumConstants.BallColor.None;
        ball3Color = results.sensor2.ballPresent ? results.sensor2.color : EnumConstants.BallColor.None;

        // Count balls
        totalBallsInRobot = 0;
        if (results.sensor0.ballPresent) totalBallsInRobot++;
        if (results.sensor1.ballPresent) totalBallsInRobot++;
        if (results.sensor2.ballPresent) totalBallsInRobot++;
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

    /**
     * Toggles auto park mode.
     * Press BACK to start parking, press again to cancel and return to manual control.
     */


    private void updateSubsystems() {
        // Manually call periodic() to ensure state machines run
        spindexer.periodic();
        shooter.periodic();

        // Update ball sensors
        robot.intakeSensorPair.update();
        robot.transferSensorPair.update();
        robot.rampSensorPair.update();
    }

    private void updateTelemetry() {
        boolean overrideRequested = gamepad1.right_stick_button;

        // ========================================
        // TOP-LEVEL KEY INFORMATION
        // ========================================
        telemetry.addData("Tracking Mode", limelight.getCurrentMode());
        telemetry.addData("Spindexer Pattern", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());

        // Motif Pattern with fallback
        if (limelight.isMotifDetected()) {
            telemetry.addData("Motif Pattern", String.format("[%s, %s, %s]",
                    LimelightConstants.motifPattern.getBallColorInSlotX(0),
                    LimelightConstants.motifPattern.getBallColorInSlotX(1),
                    LimelightConstants.motifPattern.getBallColorInSlotX(2)));
        } else {
            telemetry.addData("Motif Pattern", "Not Detected");
        }

        telemetry.addData("Spindexer Position", String.format("%d°", spindexer.getTargetPosition()));
        telemetry.addData("Turret Target", String.format("%.1f°", turret.getTargetTurretAngle()));
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
        telemetry.addData("  Shooting Mode", currentMode);
        telemetry.addData("  Field Zone", odometry.getFieldState());
        telemetry.addData("  Shooting Status", shootingValidator.getStatus(overrideRequested));
        telemetry.addData("  Balls Tracked", String.format("%d [%s, %s, %s]",
                totalBallsInRobot, ball1Color, ball2Color, ball3Color));
        telemetry.addLine("");

        // SHOOTER section
        telemetry.addLine("=== SHOOTER ===");
        telemetry.addData("  Distance to Target", String.format("%.1f in", shooter.getDistanceToTarget()));
        telemetry.addData("  Target Velocity", String.format("%.0f ticks/sec", shooter.getTargetVelocity()));
        telemetry.addData("  Actual Velocity", String.format("%.0f ticks/sec", robot.shooterMotor2.getVelocity()));
        telemetry.addData("  Target Hood Angle", String.format("%.1f deg", shooter.getTargetHoodAngle()));
        telemetry.addData("  Turret Target", String.format("%.1f° turret", turret.getTargetTurretAngle()));
        telemetry.addData("  Turret Servo", String.format("%.4f", turret.getServoPosition()));
        telemetry.addLine("");

        // SHOOTER TUNING section (only when tuning mode is active)
        if (shooter.isTuningMode()) {
            telemetry.addLine("=== SHOOTER TUNING (ACTIVE) ===");
            telemetry.addData("  Distance", String.format("%.1f in", shooter.getTuningDistance()));
            telemetry.addData("  Set Velocity", String.format("%.0f ticks/sec", shooter.getTuningSetVelocity()));
            telemetry.addData("  Set Hood Angle", String.format("%.1f deg", shooter.getTuningSetHoodAngle()));
            telemetry.addData("  Actual Velocity", String.format("%.0f ticks/sec", shooter.getCurrentVelocity()));
            telemetry.addLine("");
        }

        // SPINDEXER section
        telemetry.addLine("=== SPINDEXER ===");
        telemetry.addData("  Position (deg)", String.format("%d°", spindexer.getTargetPosition()));
        telemetry.addData("  Servo Value", String.format("%.3f", spindexer.getServoPosition()));
        telemetry.addData("  Balls [Intake|Shooter|Storage]", String.format("[%s|%s|%s]",
                SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0),
                SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1),
                SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2)));
        telemetry.addData("  Rotation Idle", spindexer.isRotationIdle());
        telemetry.addLine("");

        // SENSORS section
        telemetry.addLine("=== BALL SENSORS ===");
        DualBallDetector.Result r1 = robot.intakeSensorPair.detectBall();
        DualBallDetector.Result r2 = robot.transferSensorPair.detectBall();
        DualBallDetector.Result r3 = robot.rampSensorPair.detectBall();
        telemetry.addData("  Sensor1 (Spindexer)", String.format("%s %s %.0f%%",
                r1.ballPresent ? "BALL" : "----", r1.color, r1.confidence * 100));
        telemetry.addData("  Sensor2 (Middle)", String.format("%s %s %.0f%%",
                r2.ballPresent ? "BALL" : "----", r2.color, r2.confidence * 100));
        telemetry.addData("  Sensor3 (Outer)", String.format("%s %s %.0f%%",
                r3.ballPresent ? "BALL" : "----", r3.color, r3.confidence * 100));
        telemetry.addLine("");

        // ODOMETRY section
        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("  X Position", String.format("%.1f in", robot.pinpoint.getPosX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("  Y Position", String.format("%.1f in", robot.pinpoint.getPosY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("  Heading", String.format("%.3f rad", robot.pinpoint.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS)));
        telemetry.addLine("");

        // OTHER section
        telemetry.addLine("=== OTHER ===");
        telemetry.addData("  Alliance", RobotConstants.Robot.allianceColor);
        telemetry.addData("  IMU Calibration", "Auto on init (resetPosAndIMU)");
        telemetry.addLine("");

        // COMMAND DEBUG section
        telemetry.addLine("=== COMMAND DEBUG ===");
        telemetry.addData("  Done Rotating", spindexer.isDoneRotating());
        telemetry.addData("  Ready to Flip", spindexer.isReadyToFlip());
        telemetry.addData("  Balls In Robot", totalBallsInRobot);
        telemetry.addLine("");

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

        // Actual turret values (servo is position-controlled, no encoder feedback)
        double turretTarget = turret.getTargetTurretAngle();  // Already in turret degrees
        double servoPosition = turret.getServoPosition();

        telemetry.addData("  Turret Target", String.format("%.1f° (turret deg)", turretTarget));
        telemetry.addData("  Servo Position", String.format("%.4f", servoPosition));

        // Comparison: Expected vs What code calculated
        telemetry.addData("  Calc vs Expected Diff", String.format("%.1f°", turretTarget - expectedTurretAngle));

        telemetry.addLine("");

        telemetry.update();
    }
}