package Subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.DoubleSupplier;

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

        heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = lx * Math.cos(-heading) - ly * Math.sin(-heading);
        double rotY = lx * Math.sin(-heading) + ly * Math.cos(-heading);

        rotX *= 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        leftFrontPower = (rotY + rotX + rx) / denominator;
        leftRearPower = (rotY - rotX + rx) / denominator;
        rightFrontPower = (rotY - rotX - rx) / denominator;
        rightRearPower = (rotY + rotX - rx) / denominator;

        double mult = slowmode ? 0.25 : 1;

        robot.frontLeft.setPower(leftFrontPower * mult);
        robot.backLeft.setPower(leftRearPower * mult);
        robot.frontRight.setPower(rightFrontPower * mult);
        robot.backRight.setPower(rightRearPower * mult);
    }

}
