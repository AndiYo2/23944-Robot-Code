package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;

@TeleOp(name = "ServoPositions", group = "Tests")
public class ServoPositionTesting extends OpMode {

    public Servo spindexer, shooter;
    double spindexerPosition = SpindexerConstants.FLIPPER_POSITION_RETRACT;
    double shooterPosition = ShooterConstants.FLIPPER_POSITION_RETRACT;

    @Override
    public void init() {

        spindexer = hardwareMap.get(Servo.class, "spindexerFlipperServo");
        shooter = hardwareMap.get(Servo.class, "shooterFlipperServo");


        telemetry.addData("Status", "Initialized");
    }
    @Override
    public void loop() {


        if(gamepad1.dpadRightWasPressed()){
            spindexerPosition += .01;
        }
        if(gamepad1.dpadLeftWasPressed()){
            spindexerPosition -= .01;
        }
        if(gamepad1.rightBumperWasPressed()){
            shooterPosition += .01;
        }
        if(gamepad1.leftBumperWasPressed()){
            shooterPosition -= .01;
        }

        spindexer.setPosition(spindexerPosition);
        shooter.setPosition(shooterPosition);

        telemetry.addData("Shooter Position", shooterPosition);
        telemetry.addData("Spindexer Position", spindexerPosition);
        telemetry.update();








    }
}
