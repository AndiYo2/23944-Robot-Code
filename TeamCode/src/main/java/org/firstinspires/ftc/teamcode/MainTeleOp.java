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
    private final MecanumDrive mecanumDrive = new MecanumDrive();
    private final IntakeMotors intakeMotors = new IntakeMotors();
    private final OuttakeMotors outtakeMotors = new OuttakeMotors();
    private final TurretRotations turret = new TurretRotations();
    private final StagingServos stagingServos = new StagingServos();

    @Override
    public void runOpMode() throws InterruptedException {
        String outtakeMotorOne = "outMotor1";
        String intakeMotor = "inMotor";

        // Initialize hardware
        mecanumDrive.initDrive(hardwareMap);
        intakeMotors.initIntake(hardwareMap.get(DcMotorEx.class, intakeMotor));
        outtakeMotors.initOuttake(hardwareMap.get(DcMotorEx.class, outtakeMotorOne));
        turret.initTurret(hardwareMap);
        stagingServos.initStagingServos(hardwareMap);

        addTelemetry("Status", "Initialized");
        waitForStart();

        if (isStopRequested()) return;

        boolean released = true;

        while (opModeIsActive()) {
            handleDriving();
            handleOuttake(released);
            handleIntake();
            handleTurret();
            handleStaging();

            // Display telemetry
            addTelemetry("Robot Heading", mecanumDrive.getRobotHeading());
            addTelemetry("Status", "Running");
        }
    }

    private void handleDriving() {
        double driveY = -gamepad1.left_stick_y;
        double driveX = gamepad1.left_stick_x;
        double rotationStick = gamepad1.right_stick_x;

        if (gamepad1.options) {
            mecanumDrive.resetYaw();
        }
        if (gamepad1.circleWasPressed()) {
            mecanumDrive.changeSlowMode();
        }

        mecanumDrive.drive(driveY, driveX, rotationStick);
    }

    private void handleOuttake(boolean released) {
        if (gamepad1.right_trigger > .5) {
            outtakeMotors.shoot(1);
            released = true;
        } else if (released) {
            outtakeMotors.stopShooter();
            released = false;
        }
    }

    private void handleIntake() {
        if (gamepad1.left_bumper) {
            intakeMotors.intakeBall(-1);
        } else if (gamepad1.leftBumperWasReleased()) {
            intakeMotors.stopIntakeBall();
        }

        if (gamepad1.left_trigger > .5) {
            intakeMotors.intakeBall(1);
        } else {
            intakeMotors.stopIntakeBall();
        }
    }

    private void handleTurret() {
        if (gamepad1.dpadRightWasPressed()) {
            turret.rotate(1);
        } else if (gamepad1.dpadLeftWasPressed()) {
            turret.rotate(-1);
        } else if (gamepad1.dpadDownWasPressed()) {
            turret.stop();
        }
    }

    private void handleStaging() {
        if (gamepad1.rightBumperWasPressed()) {
            stagingServos.toggleStageServos(1);
            addTelemetry("Test", "running");
        }
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