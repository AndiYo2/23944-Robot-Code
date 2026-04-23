package tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import Constants.DriveConstants;
import Constants.EnumConstants.DriveState;
import Constants.OdometryConstants;
import Constants.RobotConstants;
import commands.DynamicReturnPathCommand;
import subsystems.MecanumDrive;
import utility.RobotHardware;

/**
 * Minimal anti-defense test TeleOp.
 *   Left Stick      = drive (field-relative)
 *   Right Stick X   = rotation
 *   LEFT_TRIGGER    = hold to engage X-Lock (resists pushing)
 *   RIGHT_TRIGGER   = press to save current pose and auto-return to it
 *                     (joystick input cancels the return)
 */
@TeleOp(name = "Defense Test", group = "Tests")
public class DefenseTestTeleOp extends CommandOpMode {

    private static final double RETURN_PATH_MAX_POWER = 0.8;

    private MecanumDrive mecanumDrive;
    private GamepadEx driverGamepad;
    private final RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;

    private Pose savedReturnPose;
    private Command activeReturnCommand;

    private boolean prevRT = false;

    @Override
    public void initialize() {
        driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        follower = pedroPathing.Constants.createFollower(hardwareMap);

        robot.pinpoint.setPosition(OdometryConstants.toPose2D(OdometryConstants.standardStartPoint));
        robot.pinpoint.update();
        follower.update();
        follower.startTeleopDrive();

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive();
        register(mecanumDrive);
    }

    @Override
    public void run() {
        super.run();

        robot.clearBulkCache();
        robot.pinpoint.update();
        robot.updateCachedPose();
        follower.update();

        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean ltActive = lt > RobotConstants.Controls.TRIGGER_THRESHOLD;
        boolean rtActive = rt > RobotConstants.Controls.TRIGGER_THRESHOLD;
        boolean joystickActive = Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND
                || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND
                || Math.abs(rx) > DriveConstants.JOYSTICK_DEADBAND;

        // Cancel an active return if driver takes over (joystick or LT).
        if (activeReturnCommand != null) {
            if (activeReturnCommand.isFinished() || joystickActive || ltActive) {
                if (!activeReturnCommand.isFinished()) {
                    activeReturnCommand.cancel();
                }
                activeReturnCommand = null;
                mecanumDrive.setAutoDriving(false);
            }
        }

        // Right trigger rising edge: snapshot pose and kick off the return path.
        if (rtActive && !prevRT && activeReturnCommand == null && !ltActive) {
            savedReturnPose = follower.getPose();
            activeReturnCommand = new DynamicReturnPathCommand(
                    follower, savedReturnPose, RETURN_PATH_MAX_POWER, true);
            mecanumDrive.setAutoDriving(true);
            schedule(activeReturnCommand);
        }
        prevRT = rtActive;

        // Left trigger held → X-Lock. Releasing drops back to FieldRelative.
        if (ltActive && mecanumDrive.getCurrentState() != DriveState.Locked
                && activeReturnCommand == null) {
            mecanumDrive.setXLock(true);
        } else if (!ltActive && mecanumDrive.getCurrentState() == DriveState.Locked) {
            mecanumDrive.setXLock(false);
        }

        mecanumDrive.drive(ly, lx, rx);

        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("=== DEFENSE TEST ===");
        telemetry.addData("Drive State", mecanumDrive.getCurrentState());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addLine();

        Pose pose = follower.getPose();
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X", "%.1f in", pose.getX());
        telemetry.addData("Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));
        telemetry.addLine();

        telemetry.addLine("=== SAVED RETURN POSE ===");
        if (savedReturnPose == null) {
            telemetry.addLine("(none — press RT to mark)");
        } else {
            telemetry.addData("X", "%.1f in", savedReturnPose.getX());
            telemetry.addData("Y", "%.1f in", savedReturnPose.getY());
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(savedReturnPose.getHeading()));
        }
        telemetry.addLine();

        telemetry.addLine("=== MOTORS ===");
        telemetry.addData("FL", "%.2f", robot.frontLeft.getPower());
        telemetry.addData("FR", "%.2f", robot.frontRight.getPower());
        telemetry.addData("BL", "%.2f", robot.backLeft.getPower());
        telemetry.addData("BR", "%.2f", robot.backRight.getPower());
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("LT (hold): X-Lock");
        telemetry.addLine("RT (press): save pose & return");
        telemetry.addLine("Any joystick: cancel return");
        telemetry.update();
    }
}
