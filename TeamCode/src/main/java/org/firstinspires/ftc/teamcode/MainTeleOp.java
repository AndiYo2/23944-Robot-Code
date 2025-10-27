package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.DrivingStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.Intake.IntakeMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingServos;
import org.firstinspires.ftc.teamcode.Outtake.OuttakeCases;
import org.firstinspires.ftc.teamcode.Outtake.OuttakeMotors;
import org.firstinspires.ftc.teamcode.Globals.BallPattern;
import org.firstinspires.ftc.teamcode.Outtake.TurretRotations;


@TeleOp
public class MainTeleOp extends LinearOpMode {


    // Create an instance of your class
    private final MecanumDrive mecanumDrive = new MecanumDrive();
    private final IntakeMotors intakeMotors = new IntakeMotors();
    private final OuttakeMotors outtakeMotors = new OuttakeMotors();
    private final TurretRotations turret = new TurretRotations();
    private final StagingServos stagingServos = new StagingServos();

    //Testing: public BallPattern pattern = new BallPattern();


    @Override
    public void runOpMode() throws InterruptedException {

        //Testing: pattern.setBallPattern(BallPattern.BallType.GREEN, BallPattern.BallType.PURPLE, BallPattern.BallType.PURPLE);

        String outtakeMotorOne = "outMotor1";
        String intakeMotor = "inMotor";


        // Initialize the MecanumDrive class with the hardware map
        mecanumDrive.initDrive(hardwareMap);
        intakeMotors.initIntake(hardwareMap.get(DcMotorEx.class, intakeMotor));
        outtakeMotors.initOuttake(hardwareMap.get(DcMotorEx.class, outtakeMotorOne));
        turret.initTurret(hardwareMap);
        stagingServos.initStagingServos(hardwareMap);

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
            double rotationStick = gamepad1.right_stick_x;

            boolean released = true;

            // Reset the IMU yaw with a button press
            if (gamepad1.options) {
                mecanumDrive.resetYaw();
            }
            //Check if circle was pressed and change slowmode accordingly
            if (gamepad1.circleWasPressed()){
                mecanumDrive.changeSlowMode();
            }

            // Outtake Motor functions

            //  Right trigger Shooting out if
            if(gamepad1.right_trigger > .5){
                outtakeMotors.shoot(1);
                released = true;
            }else if (released){
                outtakeMotors.stopShooter();
                released = false;
            }

            // Intake Motor functions, Left Trigger and bumper
            if(gamepad1.left_bumper){
                intakeMotors.intakeBall(-1 );
            }else if (gamepad1.leftBumperWasReleased()){
                intakeMotors.stopIntakeBall();
            }

            if(gamepad1.left_trigger > .5){
                intakeMotors.intakeBall(1);
            }else{
                intakeMotors.stopIntakeBall();
            }

            //Manual turret Rotator

            if(gamepad1.dpadRightWasPressed()){
                turret.rotate(1);
            }
            else if(gamepad1.dpadLeftWasPressed()){
                turret.rotate(-1);
            }else if (gamepad1.dpadDownWasPressed()){
                turret.stop();
            }

            if(gamepad1.rightBumperWasPressed()){
                stagingServos.toggleStageServos(1);
                addTelemetry("Test", "running");
            }






            // Call the drive method in the MecanumDrive class
            mecanumDrive.drive(driveY, driveX, rotationStick);

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