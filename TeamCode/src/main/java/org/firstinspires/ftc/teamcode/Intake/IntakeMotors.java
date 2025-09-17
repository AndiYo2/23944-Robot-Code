package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Timer;
import java.util.TimerTask;

public class IntakeMotors {

    // Declare variables for the motor
    double speed = .75;
    double shootTime = 5;
    boolean running = false;
    private DcMotor intakeMotor;
    Timer timer = new Timer();



    // Initialization method to map hardware
    public void init2(DcMotor intakeMotor) {
        // Retrieve and initialize motors from the hardware map
        this.intakeMotor = intakeMotor;

        // Set motor directions based on configuration
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void grabBall(){
        intakeMotor.setPower(speed);
        System.out.println("Timer Started");
        timer.schedule(new StopMotor(intakeMotor), 5000);
    }
    public void toggleIntake(){
        running = !running;
        if(running){
            intakeMotor.setPower(speed);
        }else{
            intakeMotor.setPower(0);
        }
    }
    public void intakeBall(){
        intakeMotor.setPower(speed);
    }
    public void stopIntakeBall(){
        intakeMotor.setPower(0);
    }

   /* public boolean checkBall(){
        //if there is a ball, then we categorize it and do stuff
        return false;
    }

    public void changeIntakeStatus(){
        running = !running;
    }*/

    static class StopMotor extends TimerTask{
        DcMotor motor;

        StopMotor(DcMotor motor){
            this.motor = motor;
        }

        @Override
        public void run() {
            motor.setPower(0);
            System.out.println("Timer has been completed");
        }
    }


}