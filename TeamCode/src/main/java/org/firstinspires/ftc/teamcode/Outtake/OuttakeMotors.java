package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingServos;

public class OuttakeMotors {

    private final double speed = 1;
    private DcMotorEx outtakeMotor1;
    private StagingMotors stagingMotor = new StagingMotors();
    private StagingServos stagingServos;

    private boolean flywheelRunning = false;
    private boolean autonShooting = false;

    ElapsedTime runtime = new ElapsedTime();

    public void initOuttake(HardwareMap hMap) {
        outtakeMotor1 = hMap.get(DcMotorEx.class, "outMotor1");
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void initOuttake(HardwareMap hMap, StagingServos s) {
        this.outtakeMotor1 = hMap.get(DcMotorEx.class, "outMotor1");
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        stagingServos = s;
    }


    public void shootAuton(){
        autonShooting = true;
        stagingMotor.toggleStaging();
        for (int i = 0; i < 3; i++) {
            runtime.reset();
            while(runtime.seconds() < 1) {
                //wait for 1 second
            }
            stagingMotor.invertStagingForLaunch();
            stagingServos.flip();
            runtime.reset();
            while(runtime.seconds() < .75) {
                //wait for 1 second
            }
            stagingMotor.toggleStaging();
            stagingServos.pushOutOfRamp();
        }

        autonShooting = false;


    }

    public boolean isAutonShooting(){
        return autonShooting;
    }
    /**
     * Shoots the ball: 1 for forward, -1 for reverse
     */
    public void shooterSpin(int direction) {
        outtakeMotor1.setPower(speed * direction);
    }
    public void shooterSpin(int direction, double pow) {
        outtakeMotor1.setPower(pow * direction);
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