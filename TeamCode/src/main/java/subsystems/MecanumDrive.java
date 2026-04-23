package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;

import Constants.DriveConstants;
import Constants.EnumConstants.DriveState;
import utility.RobotHardware;
import Constants.OdometryConstants;

public class MecanumDrive extends SubsystemBase {
    private RobotHardware robot;
    private boolean slowmode;
    private double dynamicSpeedMultiplier = 1.0;
    private DriveState currentState = DriveState.FieldRelative;
    private Follower activeFollower = null;
    private double headingOffset = 0;

    private Pose pose;

    public MecanumDrive() {
        this.robot = RobotHardware.getInstance();
        setPose(OdometryConstants.endingAutonPose);
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

    public void setXLock(boolean locked) {
        if (locked) {
            currentState = DriveState.Locked;
            setMotorModes(DcMotor.ZeroPowerBehavior.BRAKE);
        } else if (currentState == DriveState.Locked) {
            currentState = DriveState.FieldRelative;
        }
    }

    public void setAutoDriving(boolean autoDriving) {
        if (autoDriving) {
            currentState = DriveState.AutoDriving;
        } else if (currentState == DriveState.AutoDriving) {
            currentState = DriveState.FieldRelative;
        }
    }

    public void setDynamicSpeedMultiplier(double multiplier) {
        this.dynamicSpeedMultiplier = multiplier;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public void resetYaw(){
        headingOffset = robot.cachedHeading;
    }

    public double getRobotHeading(){
        return robot.cachedHeading - headingOffset;
    }

    private void setMotorModes(DcMotor.ZeroPowerBehavior mode) {
        robot.frontLeft.setZeroPowerBehavior(mode);
        robot.backLeft.setZeroPowerBehavior(mode);
        robot.backRight.setZeroPowerBehavior(mode);
        robot.frontRight.setZeroPowerBehavior(mode);
    }

    private void handleMotorMode(boolean driving) {
        if (driving && robot.frontLeft.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.BRAKE)) {
            setMotorModes(DcMotor.ZeroPowerBehavior.FLOAT);
        } else if (!driving && robot.frontLeft.getZeroPowerBehavior().equals(DcMotor.ZeroPowerBehavior.FLOAT)) {
            setMotorModes(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }



    public void drive(double ly, double lx, double rx) {
        if (currentState == DriveState.Idle && (Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(rx) > DriveConstants.JOYSTICK_DEADBAND)) {
            currentState = DriveState.FieldRelative;
        }

        if (currentState == DriveState.Locked) {
            setMotorModes(DcMotor.ZeroPowerBehavior.BRAKE);
            robot.frontLeft.setPower(DriveConstants.XLOCK_POWER);
            robot.backRight.setPower(DriveConstants.XLOCK_POWER);
            robot.frontRight.setPower(-DriveConstants.XLOCK_POWER);
            robot.backLeft.setPower(-DriveConstants.XLOCK_POWER);
            return;
        }

        if (currentState == DriveState.AutoDriving || currentState == DriveState.Parking) {
            setMotorModes(DcMotor.ZeroPowerBehavior.BRAKE);
            return;
        }

        handleMotorMode(Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND);

        double botHeading = getRobotHeading();

        double rotX, rotY;

        if (currentState == DriveState.FieldRelative || currentState == DriveState.SlowMode) {
            rotX = lx * Math.cos(-botHeading) - ly * Math.sin(-botHeading);
            rotY = lx * Math.sin(-botHeading) + ly * Math.cos(-botHeading);
        } else {
            rotX = lx;
            rotY = ly;
        }

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX * DriveConstants.STRAFE_COMPENSATION + rx) / denominator;
        double backLeftPower = (rotY - rotX * DriveConstants.STRAFE_COMPENSATION + rx) / denominator;
        double frontRightPower = (rotY - rotX * DriveConstants.STRAFE_COMPENSATION - rx) / denominator;
        double backRightPower = (rotY + rotX * DriveConstants.STRAFE_COMPENSATION - rx) / denominator;

        double mult = Math.min(slowmode ? DriveConstants.SLOW_MODE_MULTIPLIER : 1, dynamicSpeedMultiplier);

        robot.frontLeft.setPower(frontLeftPower * mult);
        robot.backLeft.setPower(backLeftPower * mult);
        robot.frontRight.setPower(frontRightPower * mult);
        robot.backRight.setPower(backRightPower * mult);
    }

    private void stateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                break;
            case FieldRelative:
            case SlowMode:
                break;
            case AutoDriving:
                if (activeFollower != null && !activeFollower.isBusy()) {
                    currentState = DriveState.Idle;
                    activeFollower = null;
                }
                break;
            case Parking:
                break;
            case Locked:
                break;
        }
    }
    public DriveState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
        stateMachinePeriodic();
    }

}
