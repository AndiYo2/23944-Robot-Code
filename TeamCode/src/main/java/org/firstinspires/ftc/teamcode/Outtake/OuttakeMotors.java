package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Timer;
import java.util.TimerTask;

public class OuttakeMotors {

    // Declare variables for the motor
    double speed = 1;
    double shootTime = 5;
    boolean running = false;
    private DcMotor outtakeMotor1;
    private DcMotor outtakeMotor2;
    Timer timer = new Timer();



    // Initialization method to map hardware
    public void init3(DcMotor outtakeMotor1, DcMotor outtakeMotor2) {
        // Retrieve and initialize motors from the hardware map
        this.outtakeMotor1 = outtakeMotor1;
        this.outtakeMotor2 = outtakeMotor2;

        // Set motor directions based on configuration
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

   /* public void shootBall(){
        outtakeMotor1.setPower(speed);
        System.out.println("Timer Started");
        timer.schedule(new StopMotor(outtakeMotor1), 5000);
    }*/
    public void toggleOutake(){


        running = !running;
        if(running){
            outtakeMotor1.setPower(speed);
            outtakeMotor2.setPower(speed);
        }else{
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
        }
    }
    public void shoot(){
        outtakeMotor1.setPower(speed);
        outtakeMotor2.setPower(speed);
    }
    public void notShoot(){
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
    }

    public void expunge(){
            outtakeMotor1.setPower(speed * -1);
            outtakeMotor2.setPower(speed * -1);

    }

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