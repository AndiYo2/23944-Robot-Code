package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;

@TeleOp(name = "ServoPositions", group = "Tests")
public class ServoPositionTesting extends OpMode {

    public Servo spindexer, shooter, hood;
    double spindexerPosition = SpindexerConstants.FLIPPER_POSITION_RETRACT;
    double shooterPosition = ShooterConstants.FLIPPER_POSITION_RETRACT;
    double hoodPosition = 1;

    @Override
    public void init() {

        spindexer = hardwareMap.get(Servo.class, "spindexerFlipperServo");
        shooter = hardwareMap.get(Servo.class, "shooterFlipperServo");
        hood = hardwareMap.get(Servo.class, "shooterHoodServo");


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "DPad L/R: Spindexer | Bumpers: Shooter | DPad U/D: Hood");
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
        if(gamepad1.dpadUpWasPressed()){
            hoodPosition += .01;
        }
        if(gamepad1.dpadDownWasPressed()){
            hoodPosition -= .01;
        }

        hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));

        spindexer.setPosition(spindexerPosition);
        shooter.setPosition(shooterPosition);
        hood.setPosition(hoodPosition);

        double hoodAngle = hoodPosition * 355.0;

        telemetry.addData("--- HOOD ---", "");
        telemetry.addData("Hood Position", "%.3f", hoodPosition);
        telemetry.addData("Hood Angle (calc)", "%.1f deg", hoodAngle);
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Shooter Position", "%.3f", shooterPosition);
        telemetry.addData("--- SPINDEXER ---", "");
        telemetry.addData("Spindexer Position", "%.3f", spindexerPosition);
        telemetry.addData("", "");
        telemetry.addData("Controls", "DPad U/D: Hood | Bumpers: Shooter | DPad L/R: Spindexer");
        telemetry.update();








    }
}
