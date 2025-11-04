package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingServos;

public class OuttakeMotors {

    private final double speed = 1;
    private final double necesaryFlywheelSpeed = 1000;
    private DcMotorEx outtakeMotor1;
    private StagingMotors stagingMotor = new StagingMotors();
    private StagingServos stagingServos = new StagingServos();

    private boolean flywheelRunning = false;


    public void initOuttake(DcMotorEx outtakeMotor1) {
        this.outtakeMotor1 = outtakeMotor1;
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void shootAuton(){
        shooterSpin(1);
        flywheelRunning = true;

        stagingMotor.toggleStaging();
        for(int i = 0; i < 3; i++) {
            while (true) {
                if (outtakeMotor1.getVelocity() > necesaryFlywheelSpeed) {
                    if (stagingServos.flip()) {
                        break;
                        //just to make sure it completes the flip
                    }
                }
            }
        }
        stopShooter();
        flywheelRunning = false;
        stagingMotor.stopStaging();

    }

    /**
     * Shoots the ball: 1 for forward, -1 for reverse
     */
    public void shooterSpin(int direction) {
        outtakeMotor1.setPower(speed * direction);
    }


    public boolean flywheelRunning() {
        return flywheelRunning;
    }
    /**
     * Stops the shooter motor
     */
    public void stopShooter() {
        outtakeMotor1.setPower(0);
    }

}