package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "ShooterPIDF", group = "Tests")
public class ShooterPIDFTuningTeleOp extends OpMode {

    public DcMotorEx flywheelMotor1, flywheelMotor2;
    public double highVelocity = 2700;
    public double lowVelocity = 2200;

    double currentTargetVelocity = highVelocity;

    double F = 6.4;
    double P = 5;

    double[] stepSizes = {10, 1, 0.1, 0.001 ,0.0001};
    PIDFCoefficients pidCoefficients = new PIDFCoefficients(P,0,0,F);
    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor1 = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        flywheelMotor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);




        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        flywheelMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if(gamepad1.yWasPressed()){
            if(currentTargetVelocity == highVelocity){
                currentTargetVelocity = lowVelocity;
            }else{
                currentTargetVelocity = highVelocity;
            }
        }

        if(gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if(gamepad1.dpadLeftWasPressed()){
            F -= stepSizes[stepIndex];
        }
        if(gamepad1.dpadRightWasPressed()){
            F += stepSizes[stepIndex];
        }

        if(gamepad1.dpadUpWasPressed()){
            P += stepSizes[stepIndex];
        }
        if(gamepad1.dpadDownWasPressed()){
            P -= stepSizes[stepIndex];
        }
        if(gamepad1.leftBumperWasPressed()){
            currentTargetVelocity -=100;
        }
        if(gamepad1.rightBumperWasPressed()){
            currentTargetVelocity +=100;
        }

        PIDFCoefficients pidCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);

        flywheelMotor1.setVelocity(currentTargetVelocity);
        flywheelMotor2.setVelocity(currentTargetVelocity);

        double currentVelocity2 = flywheelMotor2.getVelocity();
        double currentVelocity1 = flywheelMotor1.getVelocity();
        double error = currentTargetVelocity - currentVelocity2;

        // Diagnostic info
        int encoderPosition1 = flywheelMotor1.getCurrentPosition();
        int encoderPosition2 = flywheelMotor2.getCurrentPosition();
        double power1 = flywheelMotor1.getPower();
        double power2 = flywheelMotor2.getPower();

        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity2", currentVelocity2);
        telemetry.addData("Current Velocity1", currentVelocity1);
        telemetry.addData("Error", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Motor1 Encoder Pos", encoderPosition1);
        telemetry.addData("Motor2 Encoder Pos", encoderPosition2);
        telemetry.addData("Motor1 Power", String.format("%.2f", power1));
        telemetry.addData("Motor2 Power", String.format("%.2f", power2));
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P (D-Pad U/D)", P);
        telemetry.addData("Tuning F (D-Pad L/R)", F);
        telemetry.addData("Step Size (B Button)", stepSizes[stepIndex]);
        telemetry.update();








    }
}
