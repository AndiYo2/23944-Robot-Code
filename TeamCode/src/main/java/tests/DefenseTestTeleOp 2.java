package tests;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import commands.AggressiveReturnCommand;

import Constants.EnumConstants.DriveState;
import Constants.OdometryConstants;
import subsystems.MecanumDrive;
import utility.RobotHardware;

/**
 * Minimal anti-defense test TeleOp.
 *   Left Stick      = drive (field-relative)
 *   Right Stick X   = rotation
 *   LEFT_TRIGGER    = hold to engage X-Lock (resists pushing)
 *   RIGHT_TRIGGER   = hold to anchor: snapshot pose on press; while held,
 *                     X-Locks within AT_ANCHOR_DIST of anchor, otherwise
 *                     follows an aggressive Pedro path back at max power.
 */
@TeleOp(name = "Defense Test", group = "Tests")
public class DefenseTestTeleOp extends CommandOpMode {

    private static final double RETURN_MAX_POWER = 1.0;
    private static final double AT_ANCHOR_DIST = 0.3; // inches

    // Aggressive braking tune so short return paths don't spend the whole
    // path in the deceleration zone. Lower BRAKE_START = brake later.
    private static final double BRAKE_START = 0.15;
    private static final double BRAKE_STRENGTH = 2.0;

    // Schmitt trigger on RT so a noisy dip doesn't re-anchor mid-hold.
    private static final double RT_ENGAGE_THRESHOLD = 0.5;
    private static final double RT_RELEASE_THRESHOLD = 0.15;

    private MecanumDrive mecanumDrive;
    private final RobotHardware robot = RobotHardware.getInstance();
    private Follower follower;

    private Pose savedReturnPose;
    private boolean rtEngaged = false;
    private Command activeReturnCommand;

    private String lastAction = "init";
    private boolean driveCalledThisLoop = false;

    @Override
    public void initialize() {
        GamepadEx driverGamepad = new GamepadEx(gamepad1);
        robot.init(hardwareMap, driverGamepad);

        follower = pedroPathing.Constants.createFollower(hardwareMap);

        robot.pinpoint.setPosition(OdometryConstants.toPose2D(OdometryConstants.standardStartPoint));
        robot.pinpoint.update();
        follower.setStartingPose(OdometryConstants.standardStartPoint);
        follower.update();

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        mecanumDrive = new MecanumDrive();
        register(mecanumDrive);
    }

    @Override
    public void run() {
        robot.clearBulkCache();
        robot.pinpoint.update();
        robot.updateCachedPose();
        follower.update();

        super.run();

        follower.update();
        driveCalledThisLoop = false;

        double ly = -gamepad1.left_stick_y;
        double lx = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean ltActive = lt > 0.3;

        // RT Schmitt trigger.
        boolean wasEngaged = rtEngaged;
        if (!rtEngaged && rt > RT_ENGAGE_THRESHOLD) rtEngaged = true;
        else if (rtEngaged && rt < RT_RELEASE_THRESHOLD) rtEngaged = false;

        if (rtEngaged && !wasEngaged) {
            savedReturnPose = follower.getPose();
        }
        if (!rtEngaged && wasEngaged) {
            cancelActiveReturn();
            if (mecanumDrive.getCurrentState() == DriveState.Locked) {
                mecanumDrive.setXLock(false);
            }
        }

        if (rtEngaged && savedReturnPose != null) {
            Pose currentPose = follower.getPose();
            double dx = currentPose.getX() - savedReturnPose.getX();
            double dy = currentPose.getY() - savedReturnPose.getY();
            double distance = Math.hypot(dx, dy);

            if (distance < AT_ANCHOR_DIST) {
                cancelActiveReturn();
                if (mecanumDrive.getCurrentState() != DriveState.Locked) {
                    mecanumDrive.setXLock(true);
                }
            } else {
                if (mecanumDrive.getCurrentState() == DriveState.Locked) {
                    mecanumDrive.setXLock(false);
                }
                if (activeReturnCommand == null || activeReturnCommand.isFinished()) {
                    activeReturnCommand = new AggressiveReturnCommand(
                            follower, savedReturnPose, RETURN_MAX_POWER,
                            BRAKE_START, BRAKE_STRENGTH);
                    mecanumDrive.setAutoDriving(true);
                    schedule(activeReturnCommand);
                }
            }
        } else {
            if (ltActive && mecanumDrive.getCurrentState() != DriveState.Locked) {
                mecanumDrive.setXLock(true);
            } else if (!ltActive && mecanumDrive.getCurrentState() == DriveState.Locked) {
                mecanumDrive.setXLock(false);
            }
        }

        if (activeReturnCommand == null) {
            mecanumDrive.drive(ly, lx, rx);
            driveCalledThisLoop = true;
            lastAction = (mecanumDrive.getCurrentState() == DriveState.Locked)
                    ? "X-Lock" : "manual drive";
        } else if (activeReturnCommand.isFinished()) {
            lastAction = "path finished";
        } else {
            lastAction = "follower driving";
        }

        updateTelemetry();
    }

    private void cancelActiveReturn() {
        if (activeReturnCommand != null) {
            if (!activeReturnCommand.isFinished()) {
                activeReturnCommand.cancel();
            }
            activeReturnCommand = null;
        }
        if (mecanumDrive.getCurrentState() == DriveState.AutoDriving) {
            mecanumDrive.setAutoDriving(false);
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== DEFENSE TEST ===");
        telemetry.addData("Drive State", mecanumDrive.getCurrentState());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Last Action", lastAction);
        telemetry.addData("drive() called?", driveCalledThisLoop);
        telemetry.addData("Return Cmd Active?", activeReturnCommand != null);
        telemetry.addData("RT Engaged?", rtEngaged);
        if (savedReturnPose != null) {
            double dx = follower.getPose().getX() - savedReturnPose.getX();
            double dy = follower.getPose().getY() - savedReturnPose.getY();
            telemetry.addData("Dist to Anchor", "%.2f in", Math.hypot(dx, dy));
        }
        telemetry.addLine();

        Pose pose = follower.getPose();
        telemetry.addData("X", "%.1f in", pose.getX());
        telemetry.addData("Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f deg", Math.toDegrees(pose.getHeading()));

        telemetry.addData("FL/FR", "%.2f / %.2f", robot.frontLeft.getPower(), robot.frontRight.getPower());
        telemetry.addData("BL/BR", "%.2f / %.2f", robot.backLeft.getPower(), robot.backRight.getPower());
        telemetry.update();
    }
}
