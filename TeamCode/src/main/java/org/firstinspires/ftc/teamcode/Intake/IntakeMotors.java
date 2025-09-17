package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Timer;
import java.util.TimerTask;

public class IntakeMotors {

    // Declare variables for the motor
    double speed = .75;
    //boolean running = false;
    private DcMotor intakeMotor;



    // Initialization method to map hardware
    public void initIntake(DcMotor intakeMotor) {
        // Retrieve and initialize motors from the hardware map
        this.intakeMotor = intakeMotor;

        // Set motor directions based on configuration
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public void intakeBall(){
        intakeMotor.setPower(speed);
    }
    public void stopIntakeBall(){
        intakeMotor.setPower(0);
    }

    /*public void toggleIntake(){
        running = !running;
        if(running){
            intakeMotor.setPower(speed);
        }else{
            intakeMotor.setPower(0);
        }
    }
    */
}