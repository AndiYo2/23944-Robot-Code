package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoPositions", group = "Tests")
public class ServoPositionTesting extends OpMode {

    public Servo spindexer, shooter;
    double spindexerPosition = .5;
    double shooterPosition = .5;

    @Override
    public void init() {

        spindexer = hardwareMap.get(Servo.class, "spindexerFlipperServo");
        shooter = hardwareMap.get(Servo.class, "shooterFlipperServo");


        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {


        if(gamepad1.dpadRightWasPressed()){
            spindexerPosition += .05;
        }
        if(gamepad1.dpadLeftWasPressed()){
            spindexerPosition -= .05;
        }
        if(gamepad1.rightBumperWasPressed()){
            shooterPosition += .05;
        }
        if(gamepad1.leftBumperWasPressed()){
            shooterPosition -= .05;
        }

        spindexer.setPosition(spindexerPosition);
        shooter.setPosition(shooterPosition);

        telemetry.addData("Shooter Position", shooterPosition);
        telemetry.addData("Spindexer Position", spindexerPosition);
        telemetry.update();








    }
}
