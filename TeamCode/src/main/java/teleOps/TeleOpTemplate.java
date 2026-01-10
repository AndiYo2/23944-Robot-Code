package teleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import subsystems.*;
import utility.*;
import utility.Shooting.ShootingSequenceManager;
import utility.Shooting.ShootingValidator;

/**
 * TeleOp Template - Clean Version (No PID Tuning)
 *
 * This template provides core TeleOp functionality without PID tuning controls.
 * Use this for competition to keep the controls clean and focused.
 *
 * For PID tuning capabilities, use TeleOpTemplateTuning instead.
 */
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
    private HardwareMap hw;  // Store for park feature




    protected void initHardware(boolean isAuto) {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);
        this.hw = hardwareMap;  // Store for park feature

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

        // Mode toggles
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> limelight.toggleMode());
        new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)
            .whenPressed(() -> limelight.resetLimelight());

        // Auto Park toggle (BACK button)
        new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)
            .whenPressed(() -> toggleAutoPark());
    }

    @Override
    public void run() {
        super.run();

        // CRITICAL: Update Pinpoint odometry every loop (like in test OpMode)
        robot.pinpoint.update();

        // Update follower for position hold during parking
        if (mecanumDrive.isParking()) {
            mecanumDrive.updateFollower();
        }

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
     * Toggles auto park mode.
     * Press BACK to start parking, press again to cancel and return to manual control.
     */
    private void toggleAutoPark() {
        if (mecanumDrive.isParking()) {
            mecanumDrive.cancelPark();
        } else {
            // Get alliance-specific park position
            Pose parkTarget = (RobotConstants.UpdatableConstants.allianceColor ==
                RobotConstants.Enums.AllianceColor.Red)
                ? RobotConstants.Park.redParkZone
                : RobotConstants.Park.blueParkZone;
            mecanumDrive.parkAtPose(parkTarget, hw);
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
        // TOP-LEVEL KEY INFORMATION
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