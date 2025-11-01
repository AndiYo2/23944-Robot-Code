package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class OuttakeMotors {

    private final double speed = 1;
    private DcMotorEx outtakeMotor1;


    public void initOuttake(DcMotorEx outtakeMotor1) {
        this.outtakeMotor1 = outtakeMotor1;
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void timedShoot(int miliseconds){

    }

    /**
     * Shoots the ball: 1 for forward, -1 for reverse
     */
    public void shoot(int direction) {
        outtakeMotor1.setPower(speed * direction);
    }

    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        outtakeMotor1.setPower(0);
    }

}