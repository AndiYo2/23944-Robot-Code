package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final ElapsedTime runtime = new ElapsedTime();

    private static final double DEFAULT_SHOOTER_POWER = 0.85;

    private static final double FAR_SHOOTER_POWER= 0.9;
    private static final double CLOSE_SHOOTER_POWER = 0.7;
    private static final double SHOOTER_POWER_INCREMENT = 0.05;
    private static final double STAGING_DELAY_SECONDS = 0.1;
    private static final double FLIP_DELAY_SECONDS = 0.5;

    private double shooterPower = DEFAULT_SHOOTER_POWER;
    private boolean toggle = false;

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
            handleStagingMotor();
            displayTelemetry();
        }
    }

    private void initializeHardware() {
        mecanumDrive.initDrive(hardwareMap);
        intakeMotors.initIntake(hardwareMap);
        turret.initTurret(hardwareMap);
        stagingMotors.initStagingMotors(hardwareMap);
        stagingServos.initStagingServos(hardwareMap);
        outtakeMotors.initOuttake(hardwareMap, stagingServos);
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
        outtakeMotors.shooterSpin(1, shooterPower);
        handleTurretRotations();
        if (gamepad1.right_trigger > 0.5) {
            stagingMotors.invertStagingForLaunch();
            sleep((long) (STAGING_DELAY_SECONDS * 1000));
            stagingServos.flip();
            sleep((long) (FLIP_DELAY_SECONDS * 1000));
            stagingMotors.toggleStaging();
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
        } else if (gamepad1.left_bumper) {
            intakeMotors.intakeBall(-1);
        } else {
            intakeMotors.stopIntakeBall();
        }
        if (gamepad1.triangleWasPressed()) {
            stagingServos.pushOutOfRamp();
            addTelemetry("Pushed", "active");
        }
    }

    private void handleStagingMotor() {
        if (gamepad1.rightBumperWasPressed()) {
            stagingMotors.toggleStaging();
        }
        handleShooterPowerControls();
    }

    private void handleShooterPowerControls() {
        if (gamepad1.squareWasPressed()) {
            //togglePowerModes()
            if(toggle){
                shooterPower = CLOSE_SHOOTER_POWER;
            }else{
                shooterPower = FAR_SHOOTER_POWER;
            }
            toggle = !toggle;

        }
        if (gamepad1.dpadDownWasPressed()) {
            shooterPower = Math.max(.6, shooterPower - SHOOTER_POWER_INCREMENT);
        }
        if (gamepad1.dpadUpWasPressed()) {
            shooterPower = Math.min(1, shooterPower + SHOOTER_POWER_INCREMENT);
        }

    }

    private void displayTelemetry() {
        addTelemetry("Robot Heading", mecanumDrive.getRobotHeading());
        addTelemetry("Shooter Power", shooterPower);

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