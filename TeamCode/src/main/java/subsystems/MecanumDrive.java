package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import pedroPathing.Constants;
import utility.RobotHardware;

public class MecanumDrive implements Subsystem {
    private RobotHardware robot;
    private double leftFrontPower, leftRearPower, rightFrontPower, rightRearPower, heading;
    private boolean slowmode;

    private Pose pose;

    public MecanumDrive() {
        this.robot = RobotHardware.getInstance();
        this.pose = new Pose();
    }

    public Pose getCurrentPose() {
        return pose;
    }

    public boolean getSlowMode() {
        return slowmode;
    }

    public void setCurrentPose(Pose pose) {
        this.pose = pose;
    }

    public void setSlowMode(boolean set) {
        slowmode = set;
    }
    public void toggleSlowMode() {
        slowmode = !slowmode;
    }

    public void resetYaw(){
        robot.imu.resetYaw();
    }
    public double getRobotHeading(){
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public Follower driveToPose(Pose target, HardwareMap hardwareMap) {
        Pose a = new Pose();
        Follower follower = Constants.createFollower(hardwareMap);
        follower.activateAllPIDFs();
        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(getCurrentPose(), target))
                .setLinearHeadingInterpolation(getCurrentPose().getHeading(), target.getHeading())
                .build();
        follower.followPath(path);
        return follower;
    }

    public void stopAll() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void drive(double ly, double lx, double rx) {
        robot.telemetryManager.debug(String.format("driving %f %f %f", ly, lx, rx));
        robot.telemetryManager.update();

        double botHeading = heading;

        // Rotate the joystick input vector to be relative to the field
        double rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
        double rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);

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

}
