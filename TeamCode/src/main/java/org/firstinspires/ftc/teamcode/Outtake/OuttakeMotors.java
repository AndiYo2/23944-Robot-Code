package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Timer;
import java.util.TimerTask;

public class OuttakeMotors {

    // Declare variables for the motor
    double speed = .5;
    double shootTime = 5;
    boolean running = false;
    private DcMotor outtakeMotor;
    Timer timer = new Timer();



    // Initialization method to map hardware
    public void init3(DcMotor outtakeMotor) {
        // Retrieve and initialize motors from the hardware map
        this.outtakeMotor = outtakeMotor;

        // Set motor directions based on configuration
        outtakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void shootBall(){
        outtakeMotor.setPower(speed);
        System.out.println("Timer Started");
        timer.schedule(new StopMotor(outtakeMotor), 5000);
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