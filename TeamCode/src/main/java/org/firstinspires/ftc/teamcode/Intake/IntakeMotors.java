package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;

import java.util.Timer;
import java.util.TimerTask;

public class IntakeMotors {

    // Declare variables for the motor
    double speed = 1;
    boolean running = false;
    private DcMotorEx intakeMotor;

    StagingMotors stagingMotor = new StagingMotors();



    // Initialization method to map hardware
    public void initIntake(DcMotorEx intakeMotor) {
        // Retrieve and initialize motors from the hardware map
        this.intakeMotor = intakeMotor;

        // Set motor directions based on configuration
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }


    public void intakeBall(int positiveOrNeg){
        intakeMotor.setPower(speed * positiveOrNeg);
        if(positiveOrNeg > 0){
            stagingMotor.toggleStaging();
        }
    }
    public void stopIntakeBall(){
        intakeMotor.setPower(0);
        stagingMotor.toggleStaging();
    }


    public void toggleIntake(int positiveOrNeg){
        running = !running;
        if(running){
            intakeMotor.setPower(speed * positiveOrNeg);
            if(positiveOrNeg > 0){
                stagingMotor.toggleStaging();
            }
        }else{
            intakeMotor.setPower(0);
            stagingMotor.toggleStaging();
        }
    }

}