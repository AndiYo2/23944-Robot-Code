package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class ShooterPIDFTuningTeleOp extends OpMode {

    public DcMotorEx flywheelMotor1, flywheelMotor2;
    public double highVelocity = 1500;
    public double lowVelocity = 1000;

    double currentTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

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
        PIDFCoefficients pidCoefficients = new PIDFCoefficients(P,0,0,F);
        flywheelMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);
        flywheelMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidCoefficients);

        flywheelMotor1.setVelocity(currentTargetVelocity);
        flywheelMotor2.setVelocity(currentTargetVelocity);

        double currentVelocity = flywheelMotor1.getVelocity();
        double error = currentTargetVelocity - currentVelocity;


        telemetry.addData("Target Velocity", currentTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", currentVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad U/D) ",  P);
        telemetry.addData("Tuning F", "%.4f (D-Pad L/R) ",  F);
        telemetry.addData("Step Size", "%.4f (B Button) ",  F);
        telemetry.update();








    }
}
