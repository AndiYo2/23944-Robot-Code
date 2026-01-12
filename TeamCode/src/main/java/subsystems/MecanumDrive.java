package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Constants.DriveConstants;
import Constants.EnumConstants.DriveState;
import Constants.RobotHardware;
import Constants.OdometryConstants;

public class MecanumDrive implements Subsystem {
    private RobotHardware robot;
    private boolean slowmode;
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

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public void resetYaw(){
        // Store current heading as offset - makes current direction the new "forward"
        // This preserves position (for shooter aiming) while resetting driver orientation
        robot.pinpoint.update();
        headingOffset = robot.pinpoint.getHeading(AngleUnit.RADIANS);
    }

    public double getRobotHeading(){
        // Return heading relative to the reset point (subtract offset)
        robot.pinpoint.update();
        return robot.pinpoint.getHeading(AngleUnit.RADIANS) - headingOffset;
    }

    /**
     * Updates the follower for position hold during parking.
     * Call this in the main loop when parking is active.
     */
    public void updateFollower() {
        if (activeFollower != null) {
            activeFollower.update();
        }
    }

    public void stop() {
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
    }

    public void drive(double ly, double lx, double rx) {
        // Auto-transition from Idle when joystick input detected
        if (currentState == DriveState.Idle && (Math.abs(ly) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(lx) > DriveConstants.JOYSTICK_DEADBAND || Math.abs(rx) > DriveConstants.JOYSTICK_DEADBAND)) {
            currentState = DriveState.FieldRelative;
        }

        // Skip driving if in AutoDriving, Parking, or Locked state
        if (currentState == DriveState.AutoDriving || currentState == DriveState.Parking || currentState == DriveState.Locked) {
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

        rotX = rotX * DriveConstants.STRAFE_COMPENSATION;  // Counteract imperfect strafing

        // Calculate and normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        double mult = slowmode ? DriveConstants.SLOW_MODE_MULTIPLIER : 1;

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

    public boolean isIdle() {
        return currentState == DriveState.Idle;
    }

    @Override
    public void periodic() {
        stateMachinePeriodic();
    }

}
