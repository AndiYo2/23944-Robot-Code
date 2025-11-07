package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivingStuff.MecanumDrive;
import org.firstinspires.ftc.teamcode.Intake.IntakeMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingServos;
import org.firstinspires.ftc.teamcode.Outtake.OuttakeMotors;
import org.firstinspires.ftc.teamcode.Outtake.TurretRotations;

@TeleOp
public class MainTeleOp extends LinearOpMode {
    private final MecanumDrive mecanumDrive = new MecanumDrive();
    private final IntakeMotors intakeMotors = new IntakeMotors();
    private final OuttakeMotors outtakeMotors = new OuttakeMotors();
    private final TurretRotations turret = new TurretRotations();
    private final StagingServos stagingServos = new StagingServos();
    private final StagingMotors stagingMotors = new StagingMotors();

    private static final double STAGING_DELAY_SECONDS = 0.1;
    private static final double FLIP_DELAY_SECONDS = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        addTelemetry("Status", "Initialized");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            handleDriving();
            handleShooter();
            handleIntake();
            displayTelemetry();
            handleShooterPowerControls();
            handleStaging();
        }

    }

    private void initializeHardware() {
        mecanumDrive.initDrive(hardwareMap);
        intakeMotors.initIntake(hardwareMap);
        turret.initTurret(hardwareMap);
        stagingMotors.initStagingMotors(hardwareMap);
        stagingServos.initStagingServos(hardwareMap);
        outtakeMotors.initOuttake(hardwareMap, stagingServos, intakeMotors);
    }

    private void handleDriving() {
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        if (gamepad1.options) {
            mecanumDrive.resetYaw();
        }
        if (gamepad1.circleWasPressed()) {
            mecanumDrive.changeSlowMode();
        }

        mecanumDrive.drive(driveY, driveX, rotation);
    }

    private void handleShooter() {
        outtakeMotors.shooterSpin(1);
        handleTurretRotations();
        if (gamepad1.right_trigger > 0.5) {
            sleep((long) (STAGING_DELAY_SECONDS * 1000));
            stagingServos.flip();
            sleep((long) (FLIP_DELAY_SECONDS * 1000));
        }
        if(gamepad1.rightBumperWasPressed()){
            stagingMotors.invertStagingForLaunchAuto();
        }
    }

    private  void handleTurretRotations(){
        if(gamepad1.dpad_left && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            turret.rotate(-1);
        }
        else if(gamepad1.dpad_right && !gamepad1.dpad_up && !gamepad1.dpad_down) {
            turret.rotate(1);
        }else{
            turret.stop();
        }
    }

    private void handleIntake() {
        if (gamepad1.left_trigger > 0.5) {
            intakeMotors.intakeBall(1);
        }else {
            intakeMotors.stopIntakeBall();
        }
        if (gamepad1.triangleWasPressed()) {
            stagingServos.pushOutOfRamp();
            addTelemetry("Pushed", "active");
        }
    }



    private void handleShooterPowerControls() {
        if (gamepad1.squareWasPressed()) {
            outtakeMotors.shooterPowerToggle();
        }
        if (gamepad1.dpadDownWasPressed() && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            outtakeMotors.decreaseFlywheelSpeed();
        }
        if (gamepad1.dpadUpWasPressed() && !gamepad1.dpad_left && !gamepad1.dpad_right) {
            outtakeMotors.increaseFlywheelSpeed();
        }
    }

    private void handleStaging(){
        if(gamepad1.leftBumperWasPressed()){
            stagingMotors.runStaging();
        }
        if(gamepad1.leftBumperWasReleased()){
            stagingMotors.stopStaging();
        }
    }

    private void displayTelemetry() {
        addTelemetry("Robot Heading", mecanumDrive.getRobotHeading());
        addTelemetry("Shooter Power", outtakeMotors.getFlywheelSpeed());

    }

    private void addTelemetry(String label, String value) {
        telemetry.addData(label, value);
        telemetry.update();
    }

    private void addTelemetry(String label, double value) {
        telemetry.addData(label, value);
        telemetry.update();
    }
}