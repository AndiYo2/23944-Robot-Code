package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivingStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.Intake.IntakeMotors;

@TeleOp
public class MainTeleOp extends LinearOpMode {

    // Create an instance of your new MecanumDrive class
    private MecanumDrive mecanumDrive = new MecanumDrive();
    private IntakeMotors intakeMotors = new IntakeMotors();

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize the MecanumDrive class with the hardware map
        mecanumDrive.init(hardwareMap);
        intakeMotors.init(hardwareMap);

        //Telemetry add time
        addTelemetry("Status", "Initialized");

        // Wait for the initialization of the robot, bc somewhere this has to be present
        // So we made it here!
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Read driver inputs from the gamepad
            double driveY = -gamepad1.left_stick_y;
            double driveX = gamepad1.left_stick_x;
            double rotationStick = -gamepad1.right_stick_x;

            // Reset the IMU yaw with a button press
            if (gamepad1.options) {
                mecanumDrive.resetYaw();
            }
            //Check if circle was pressed and change slowmode accordingly
            if (gamepad1.circleWasPressed()){
                mecanumDrive.changeSlowMode();
            }
            if (gamepad1.triangleWasPressed()){
                intakeMotors.changeIntakeStatus();
            }

            // Call the drive method in the MecanumDrive class
            mecanumDrive.drive(driveY, driveX, rotationStick);

            intakeMotors.grabBall();

            // Display telemetry data
            addTelemetry("Robot Heading", mecanumDrive.getRobotHeading());
            addTelemetry("Status", "Running");
        }
    }

    //This is where we can add our telemetry data
    public void addTelemetry(String label, String value){
        telemetry.addData(label, value);
        telemetry.update();
    }
    public void addTelemetry(String label, double value){
        telemetry.addData(label, value);
        telemetry.update();
    }
}