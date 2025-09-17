package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeMotors {

    // Declare variables for the motors
    double speed = 1;
    /*boolean running = false;*/
    private DcMotor outtakeMotor1;
    private DcMotor outtakeMotor2;


    // Initialization method to map hardware
    public void initOuttake(DcMotor outtakeMotor1, DcMotor outtakeMotor2) {
        // Retrieve and initialize motors from the hardware map
        this.outtakeMotor1 = outtakeMotor1;
        this.outtakeMotor2 = outtakeMotor2;

        // Set motor directions based on configuration
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //Shoots the ball, if 1 is passed, shoots forward,
    // if -1 is passed, expunges the ball
    public void shoot(int direction){
        outtakeMotor1.setPower(speed * direction);
        outtakeMotor2.setPower(speed * direction);
    }
    //stops the shooter
    public void stopShooter(){
        outtakeMotor1.setPower(0);
        outtakeMotor2.setPower(0);
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