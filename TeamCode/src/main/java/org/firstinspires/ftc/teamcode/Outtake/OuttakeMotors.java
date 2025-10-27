package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeMotors {

    // Declare variables for the motors
    double speed = 1;
    /*boolean running = false;*/
    private DcMotorEx outtakeMotor1;


    // Initialization method to map hardware
    public void initOuttake(DcMotorEx outtakeMotor1) {
        // Retrieve and initialize motors from the hardware map
        this.outtakeMotor1 = outtakeMotor1;

        // Set motor directions based on configuration
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    //Shoots the ball, if 1 is passed, shoots forward,
    // if -1 is passed, expunges the ball
    public void shoot(int direction){
        outtakeMotor1.setPower(speed * direction);
    }
    //stops the shooter
    public void stopShooter(){
        outtakeMotor1.setPower(0);
    }

    /*
    Use if you want a continuous shooter motion, used as a toggle
    if its not being used, comment it to save space
    public void toggleOuttake(){

        running = !running;
        if(running){
            outtakeMotor1.setPower(speed);
            outtakeMotor2.setPower(speed);
        }else {
            outtakeMotor1.setPower(0);
            outtakeMotor2.setPower(0);
        }
    }
    */

}