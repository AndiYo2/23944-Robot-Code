package tests;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import Constants.DriveConstants;
import Constants.FieldMap;
import Constants.NamingConstants;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import subsystems.MecanumDrive;
import utility.RobotHardware;

/**
 * Drive + Odometry diagnostic TeleOp.
 * Combines field-centric mecanum drive with pinpoint position testing.
 *
 * CONTROLS:
 *   Left Stick     = drive (field-relative)
 *   Right Stick X  = rotation
 *   B              = toggle slow mode
 *   Start          = reset yaw
 *   A              = toggle field-relative / robot-relative
 *   D-Pad Up/Down  = park servo +/- 0.01
 *   Y              = toggle park servo extend/retract
 */
@TeleOp(name = "Drive Test", group = "Tests")
public class DriveTest extends CommandOpMode {

    private MecanumDrive mecanumDrive;
    private GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();

    private Servo parkServo;
    private double parkPosition = 0.5;
    private boolean parkExtended = false;

    private double robotX, robotY, robotHeading;
    private boolean fieldRelative = true;
    private boolean prevA = false;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        robot.pinpoint.setPosition(OdometryConstants.toPose2D(OdometryConstants.standardStartPoint));
        robot.pinpoint.update();

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        parkServo = hardwareMap.get(Servo.class, NamingConstants.Drivetrain.parkServo);
        parkServo.setPosition(parkPosition);

        mecanumDrive = new MecanumDrive();
        register(mecanumDrive);

        robotX = OdometryConstants.standardStartPoint.getX();
        robotY = OdometryConstants.standardStartPoint.getY();
        robotHeading = OdometryConstants.standardStartPoint.getHeading();
    }

    @Override
    public void run() {
        super.run();

        robot.clearBulkCache();
        robot.pinpoint.update();
        robot.updateCachedPose();

        robotX = robot.cachedPoseX;
        robotY = robot.cachedPoseY;
        robotHeading = robot.cachedHeading;

        // Toggle field-relative on A press
        if (gamepad1.a && !prevA) {
            fieldRelative = !fieldRelative;
        }
        prevA = gamepad1.a;

        // Reset yaw on Start
        if (gamepad1.start) {
            mecanumDrive.resetYaw();
        }

        // Park servo tuning
        driverGamepad.readButtons();
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            parkPosition = Math.min(1.0, parkPosition + 0.01);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            parkPosition = Math.max(0.0, parkPosition - 0.01);
        }
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.Y)) {
            parkExtended = !parkExtended;
            parkPosition = parkExtended ? DriveConstants.PARK_SERVO_EXTEND : DriveConstants.PARK_SERVO_RETRACT;
        }
        parkServo.setPosition(parkPosition);

        // Toggle slow mode on B
        // (handled via subsystem, but we track for telemetry)

        // Drive
        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (fieldRelative) {
            mecanumDrive.drive(ly, lx, rx);
        } else {
            // Robot-relative: bypass field transform, write motors directly
            double rotX = lx * DriveConstants.STRAFE_COMPENSATION;
            double denominator = Math.max(Math.abs(ly) + Math.abs(rotX) + Math.abs(rx), 1);
            robot.frontLeft.setPower((ly + rotX + rx) / denominator);
            robot.backLeft.setPower((ly - rotX + rx) / denominator);
            robot.frontRight.setPower((ly - rotX - rx) / denominator);
            robot.backRight.setPower((ly + rotX - rx) / denominator);
        }

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== DRIVE TEST ===");
        telemetry.addData("Mode", fieldRelative ? "Field-Relative" : "Robot-Relative");
        telemetry.addData("Drive State", mecanumDrive.getCurrentState());
        telemetry.addLine();

        // Position
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.1f in", robotX);
        telemetry.addData("Y", "%.1f in", robotY);
        telemetry.addData("Heading", "%.1f deg (%.2f rad)",
                Math.toDegrees(robotHeading), robotHeading);
        telemetry.addData("Center Zone", getZoneName(FieldMap.getPosition(robotX, robotY)));
        telemetry.addLine();

        // Corners
        telemetry.addLine("=== CORNERS ===");
        double halfSize = RobotConstants.Robot.HALF_SIZE;
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);

        double[][] offsets = {
            { halfSize,  halfSize},  // Front-Right
            { halfSize, -halfSize},  // Front-Left
            {-halfSize,  halfSize},  // Back-Right
            {-halfSize, -halfSize}   // Back-Left
        };
        String[] names = {"FR", "FL", "BR", "BL"};

        boolean anyInShootZone = false;
        for (int i = 0; i < 4; i++) {
            double cx = robotX + (offsets[i][0] * cosH - offsets[i][1] * sinH);
            double cy = robotY + (offsets[i][0] * sinH + offsets[i][1] * cosH);
            char zone = FieldMap.getPosition(cx, cy);
            telemetry.addData(names[i], "(%.1f, %.1f) %s", cx, cy, getZoneName(zone));
            if (zone == 'S') anyInShootZone = true;
        }
        telemetry.addData("In Shooting Zone?", anyInShootZone ? "YES" : "NO");
        telemetry.addLine();

        // Motor powers
        telemetry.addLine("=== MOTORS ===");
        telemetry.addData("FL", "%.2f", robot.frontLeft.getPower());
        telemetry.addData("FR", "%.2f", robot.frontRight.getPower());
        telemetry.addData("BL", "%.2f", robot.backLeft.getPower());
        telemetry.addData("BR", "%.2f", robot.backRight.getPower());
        telemetry.addLine();

        // Velocity
        telemetry.addLine("=== VELOCITY ===");
        telemetry.addData("Vel X", "%.1f in/s", robot.cachedVelX);
        telemetry.addData("Vel Y", "%.1f in/s", robot.cachedVelY);
        telemetry.addData("Heading Vel", "%.2f rad/s", robot.cachedHeadingVel);
        telemetry.addLine();

        telemetry.addLine("=== PARK SERVO ===");
        telemetry.addData("Position", parkPosition);
        telemetry.addData("State", parkExtended ? "EXTENDED" : "RETRACTED");
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Toggle Field/Robot Relative");
        telemetry.addLine("B: Toggle Slow Mode");
        telemetry.addLine("Start: Reset Yaw");
        telemetry.addLine("DPad Up/Down: Park Servo +/- 0.01");
        telemetry.addLine("Y: Toggle Park Extend/Retract");

        telemetry.update();
    }

    private String getZoneName(char zone) {
        switch (zone) {
            case 'N': return "Normal";
            case 'S': return "Shoot Zone";
            case 'R': return "Red Zone";
            case 'B': return "Blue Zone";
            case 'G': return "Goal";
            default:  return "Unknown";
        }
    }
}
