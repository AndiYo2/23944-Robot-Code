package org.firstinspires.ftc.teamcode.DrivingStuff;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class MecanumDrive {

    // Declare private variables for the motors and IMU
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private IMU imu;

    private boolean isSlow = false;

    public MecanumDrive() {
        // Constructor, can be left empty
    }

    // Initialization method to map hardware
    public void init(HardwareMap hardwareMap) {
        // Retrieve and initialize motors from the hardware map
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Set motor directions based on configuration
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve and initialize the IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
    }




    // Method to drive the robot using field-centric control
    public void drive(double y, double x, double rx) {

        // Retrieve the robot's heading from the IMU
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = orientation.getYaw(AngleUnit.RADIANS);

        // Rotate the joystick input vector to be relative to the field
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Calculate and normalize motor powers
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;


        // if slowmode button was pressed, then bot should drive 4x slower
        if(isSlow){
            frontLeftPower /= 4;
            frontRightPower /= 4;
            backLeftPower /= 4;
            backRightPower /= 4;
        }


        // Set motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    // Method to reset the IMU yaw
    public void resetYaw() {
        imu.resetYaw();
    }
    // Method to change slowMode back and forth
    public void changeSlowMode(){
        isSlow = !isSlow;
    }

    // Method to get the robot's heading for telemetry
    public double getRobotHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}