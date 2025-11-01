package org.firstinspires.ftc.teamcode.InternalSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Timer;
import java.util.TimerTask;

public class StagingMotors {

    // Declare variables for the motor
    double speed = 1;
    boolean running = false;
    private DcMotorEx stagingMotor;



    // Initialization method to map hardware
    public void initStagingMotors(DcMotorEx sMotor) {
        // Retrieve and initialize motors from the hardware map
        this.stagingMotor = sMotor;

        // Set motor directions based on configuration
        stagingMotor.setDirection(DcMotorEx.Direction.FORWARD);
    }




    public void toggleStaging(int positiveOrNeg){
        running = !running;
        if(running){
            stagingMotor.setPower(speed * positiveOrNeg);
        }else{
            stagingMotor.setPower(0);
        }
    }

}