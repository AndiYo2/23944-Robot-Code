package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.Constants;
import utility.RobotConstants;
import utility.RobotConstants.Enums.DriveState;
import utility.RobotHardware;

public class MecanumDrive implements Subsystem {
    private RobotHardware robot;
    private double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading;
    private boolean slowmode;
    private DriveState currentState = DriveState.FieldRelative;
    private Follower activeFollower = null;

    private Pose pose;

    public MecanumDrive() {
        this.robot = RobotHardware.getInstance();
        setPose(RobotConstants.UpdatableConstants.endingAutonPose);
    }

    public Pose getCurrentPose() {
        return pose;
    }

    public void toggleSlowMode() {
        slowmode = !slowmode;
        if (slowmode) {
            currentState = DriveState.SlowMode;
        } else {
            currentState = DriveState.FieldRelative;
        }
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public void resetYaw(){
        robot.pinpoint.resetPosAndIMU();
    }
    public double getRobotHeading(){
        robot.pinpoint.update();
        return robot.pinpoint.getHeading(AngleUnit.RADIANS);

    }

    public Follower driveToPose(Pose target, HardwareMap hardwareMap) {
        currentState = DriveState.AutoDriving;
        Follower follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPose(), target))
                .setLinearHeadingInterpolation(getCurrentPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path);
        activeFollower = follower;
        return follower;
    }

    public void stop() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void drive(double ly, double lx, double rx) {
        // Auto-transition from Idle when joystick input detected
        if (currentState == DriveState.Idle && (Math.abs(ly) > 0.01 || Math.abs(lx) > 0.01 || Math.abs(rx) > 0.01)) {
            currentState = DriveState.FieldRelative;
        }

        // Skip driving if in AutoDriving or Locked state
        if (currentState == DriveState.AutoDriving || currentState == DriveState.Locked) {
            return;
        }

        double botHeading = getRobotHeading();

        double rotX, rotY;

        // Apply field-relative transformation only in field-relative modes
        if (currentState == DriveState.FieldRelative || currentState == DriveState.SlowMode) {
            // Rotate the joystick input vector to be relative to the field
            rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
            rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);
        } else {
            // Robot-relative mode - no rotation
            rotX = lx;
            rotY = ly;
        }

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Calculate and normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        double mult = slowmode ? 0.25 : 1;

        robot.frontLeft.setPower(frontLeftPower * mult);
        robot.backLeft.setPower(backLeftPower * mult);
        robot.frontRight.setPower(frontRightPower * mult);
        robot.backRight.setPower(backRightPower * mult);
    }

    private void stateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                // Not moving
                break;
            case FieldRelative:
            case SlowMode:
                // Handled by drive() method
                break;
            case AutoDriving:
                // Check if autonomous navigation is complete
                if (activeFollower != null && !activeFollower.isBusy()) {
                    currentState = DriveState.Idle;
                    activeFollower = null;
                }
                break;
            case Locked:
                // X-pattern wheel lock (anti-defense)
                robot.frontLeft.setPower(1.0);
                robot.backLeft.setPower(-1.0);
                robot.frontRight.setPower(-1.0);
                robot.backRight.setPower(1.0);
                break;
        }
    }

    // ****** STATE CONTROL ******

    public void lockRobot() {
        currentState = DriveState.Locked;
    }

    public void unlockRobot() {
        currentState = DriveState.FieldRelative;
    }

    // ****** STATE QUERIES ******
    public DriveState getCurrentState() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == DriveState.Idle;
    }

    @Override
    public void periodic() {
        stateMachinePeriodic();
    }

}
