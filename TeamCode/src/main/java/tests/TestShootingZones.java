package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import subsystems.MecanumDrive;
import subsystems.Shooter;
import utility.RobotHardware;
import utility.ShootingValidator;
import utility.RobotConstants.Enums.FieldState;

/**
 * TestShootingZones
 *
 * Tests the shooting zone restrictions and override functionality.
 *
 * Test Cases:
 * 1. Drive robot inside shooting zone, press shoot
 *    - Should allow shooting
 *    - Telemetry shows "ALLOWED"
 * 2. Drive robot outside shooting zone, press shoot
 *    - Should block shooting
 *    - Telemetry shows "BLOCKED"
 *    - Controller rumbles
 * 3. Outside zone, click RIGHT_STICK_BUTTON, press shoot
 *    - Should allow shooting (override active)
 *    - Telemetry shows "OVERRIDE ACTIVE"
 *
 * Controls:
 * - Left Stick: Drive
 * - Right Stick X: Rotate
 * - Left Bumper: Attempt to shoot
 * - RIGHT_STICK_BUTTON (click right stick): Override shooting restriction
 * - A Button: Reset odometry position
 */
@TeleOp(name = "Test: Shooting Zones", group = "Tests")
public class TestShootingZones extends OpMode {

    private RobotHardware robot;
    private ShootingValidator shootingValidator;
    private Shooter shooter;
    private MecanumDrive drive;

    private int allowedShots = 0;
    private int blockedShots = 0;
    private int overrideShots = 0;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        // Initialize subsystems
        shooter = new Shooter();
        drive = new MecanumDrive();

        // Initialize shooting validator
        shootingValidator = new ShootingValidator(shooter, telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Left Bumper: Attempt shoot");
        telemetry.addLine("DPAD_LEFT + RIGHT_BUMPER: Override");
        telemetry.addLine("A: Reset odometry");
        telemetry.update();
    }

    @Override
    public void start() {
        allowedShots = 0;
        blockedShots = 0;
        overrideShots = 0;
    }

    @Override
    public void loop() {
        // Update subsystems
        robot.pinpoint.update();
        shooter.periodic();
        drive.periodic();

        // Handle odometry reset
        if (gamepad1.a) {
            robot.pinpoint.resetPosAndIMU();
        }

        // Drive control (field-relative mecanum)
        double y = -gamepad1.left_stick_y;  // Forward/backward
        double x = gamepad1.left_stick_x;   // Strafe left/right
        double rx = gamepad1.right_stick_x; // Rotation

        // Apply drive with field-relative control
        Pose2D currentPose = robot.pinpoint.getPosition();
        double heading = currentPose.getHeading(AngleUnit.RADIANS);

        // Convert field-relative to robot-relative
        double rotatedX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotatedY = x * Math.sin(-heading) + y * Math.cos(-heading);

        // Apply to motors
        double denominator = Math.max(Math.abs(rotatedY) + Math.abs(rotatedX) + Math.abs(rx), 1);
        double frontLeftPower = (rotatedY + rotatedX + rx) / denominator;
        double backLeftPower = (rotatedY - rotatedX + rx) / denominator;
        double frontRightPower = (rotatedY - rotatedX - rx) / denominator;
        double backRightPower = (rotatedY + rotatedX - rx) / denominator;

        robot.frontLeft.setPower(frontLeftPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);

        // Check for override input (click right stick)
        boolean overrideRequested = gamepad1.right_stick_button;

        // Attempt to shoot
        if (gamepad1.left_bumper) {
            boolean canShoot = shootingValidator.canShoot(overrideRequested);

            if (canShoot) {
                // Shoot allowed
                shooter.triggerShot();

                // Track statistics
                if (shootingValidator.isInShootingZone()) {
                    allowedShots++;
                } else {
                    overrideShots++;
                }
            } else {
                // Shooting blocked - rumble controller
                gamepad1.rumble(200);
                blockedShots++;
            }
        }

        // Build telemetry
        displayTelemetry(overrideRequested);
    }

    private void displayTelemetry(boolean overrideRequested) {
        Pose2D currentPose = robot.pinpoint.getPosition();
        double x = currentPose.getX(DistanceUnit.INCH);
        double y = currentPose.getY(DistanceUnit.INCH);
        double heading = currentPose.getHeading(AngleUnit.DEGREES);
        FieldState fieldState = shooter.getFieldState();

        telemetry.addData("=== POSITION ===", "");
        telemetry.addData("X", "%.1f in", x);
        telemetry.addData("Y", "%.1f in", y);
        telemetry.addData("Heading", "%.1f°", heading);
        telemetry.addData("", "");

        telemetry.addData("=== FIELD STATE ===", "");
        telemetry.addData("Current Zone", fieldState.toString());
        telemetry.addData("In Shooting Zone?", shootingValidator.isInShootingZone() ? "YES" : "NO");
        telemetry.addData("", "");

        telemetry.addData("=== SHOOTING STATUS ===", "");
        String shootingStatus = shootingValidator.getStatus(overrideRequested);

        // Color-code status
        if (shootingStatus.equals("ALLOWED")) {
            telemetry.addData("Status", "\ud83d\udfe2 " + shootingStatus); // Green circle
        } else if (shootingStatus.equals("OVERRIDE")) {
            telemetry.addData("Status", "\ud83d\udfe1 " + shootingStatus); // Yellow circle
        } else {
            telemetry.addData("Status", "\ud83d\udd34 " + shootingStatus); // Red circle
        }

        telemetry.addData("Override Active?", overrideRequested ? "YES" : "NO");
        telemetry.addData("", "");

        telemetry.addData("=== STATISTICS ===", "");
        telemetry.addData("Allowed Shots", allowedShots);
        telemetry.addData("Blocked Shots", blockedShots);
        telemetry.addData("Override Shots", overrideShots);
        telemetry.addData("", "");

        telemetry.addData("=== TEST RESULTS ===", "");

        // Test 1: Inside zone allows shooting
        if (allowedShots > 0) {
            telemetry.addData("Test 1", "\u2705 PASS - Shot allowed in zone (%d times)", allowedShots);
        } else {
            telemetry.addData("Test 1", "Drive into shooting zone and press LEFT_BUMPER");
        }

        // Test 2: Outside zone blocks shooting
        if (blockedShots > 0) {
            telemetry.addData("Test 2", "\u2705 PASS - Shot blocked outside zone (%d times)", blockedShots);
        } else {
            telemetry.addData("Test 2", "Drive outside zone and press LEFT_BUMPER");
        }

        // Test 3: Override allows shooting outside zone
        if (overrideShots > 0) {
            telemetry.addData("Test 3", "\u2705 PASS - Override worked (%d times)", overrideShots);
        } else {
            telemetry.addData("Test 3", "Outside zone: Click RIGHT_STICK, press LEFT_BUMPER");
        }

        telemetry.addData("", "");
        telemetry.addData("Controls", "LEFT_BUMPER to shoot");
        telemetry.addData("", "Click RIGHT_STICK to override");

        telemetry.update();
    }

    @Override
    public void stop() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }
}
