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
    private double headingOffset = 0; // Offset applied when driver resets yaw

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

    public void setDynamicSpeedMultiplier(double multiplier) {
        this.dynamicSpeedMultiplier = multiplier;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public void resetYaw(){
        // Store current cached heading as offset - makes current direction the new "forward"
        // This preserves position (for shooter aiming) while resetting driver orientation
        headingOffset = robot.cachedHeading;
    }

    public double getRobotHeading(){
        // Return heading relative to the reset point (subtract offset)
        // Cached heading is updated once at the start of each loop in TeleOpTemplate.run()
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
        // Auto-transition from Idle when joystick input detected
        if (currentState == DriveState.Idle && (Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(rx) > DriveConstants.JOYSTICK_DEADBAND)) {
            currentState = DriveState.FieldRelative;
        }

        // Skip driving if in AutoDriving, Parking, or Locked state
        if (currentState == DriveState.AutoDriving || currentState == DriveState.Parking || currentState == DriveState.Locked) {
            setMotorModes(DcMotor.ZeroPowerBehavior.BRAKE);
            return;
        }

        handleMotorMode(Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND);

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

        rotX = rotX * DriveConstants.STRAFE_COMPENSATION;  // Counteract imperfect strafing

        // Calculate and normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

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
                // Check if autonomous navigation is complete
                if (activeFollower != null && !activeFollower.isBusy()) {
                    currentState = DriveState.Idle;
                    activeFollower = null;
                }
                break;
            case Parking:
                // Keep follower active for position hold - don't transition to Idle
                // Position hold is maintained by calling updateFollower() from TeleOp loop
                break;
        }
    }

    // ****** STATE QUERIES ******
    public DriveState getCurrentState() {
        return currentState;
    }

    @Override
    public void periodic() {
        stateMachinePeriodic();
    }

}
