package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Intake.IntakeMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingMotors;
import org.firstinspires.ftc.teamcode.InternalSystems.StagingServos;

public class OuttakeMotors {

    private final double speed = 1;
    private DcMotorEx outtakeMotor1;
    private StagingMotors stagingMotor = new StagingMotors();
    private IntakeMotors intakeMotors = new IntakeMotors();
    private StagingServos stagingServos;



    private static final double FAR_SHOOTER_POWER= 0.9;
    private static final double CLOSE_SHOOTER_POWER = 0.7;
    private static final double SHOOTER_POWER_INCREMENT = 0.05;

    private boolean flywheelRunning = false;
    private boolean autonShooting = false;
    private boolean toggle = false;

    private double power = 0.7;

    ElapsedTime runtime = new ElapsedTime();

    public void initOuttake(HardwareMap hMap) {
        outtakeMotor1 = hMap.get(DcMotorEx.class, "outMotor1");
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void initOuttake(HardwareMap hMap, StagingServos s, IntakeMotors i) {
        this.outtakeMotor1 = hMap.get(DcMotorEx.class, "outMotor1");
        outtakeMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        stagingServos = s;
        intakeMotors = i;
    }


    // MAKE INTO FULL INSTRUCTUON LIST NOT FOR LOOP

    public void shootAuton(){
        autonShooting = true;

        stagingServos.flip();
        waitT(.25);
        intakeMotors.intakeBall(1);
        stagingServos.halfFlip();
        waitT(.5);
        intakeMotors.stopIntakeBall();
        stagingServos.flip();
        waitT(.25);
        intakeMotors.intakeBall(1);
        stagingServos.pushOutOfRamp();
        waitT(.25);
        intakeMotors.stopIntakeBall();
        stagingServos.flip();
        waitT(.1);

        autonShooting = false;
        intakeMotors.intakeBall(1);

    }
    public void shootAutonSecondary(){
        autonShooting = true;

        intakeMotors.stopIntakeBall();
        stagingMotor.invertStagingForLaunchAuto();
        waitT(.1);
        stagingServos.flip();
        waitT(.25);
        intakeMotors.intakeBall(1);
        stagingServos.halfFlip();
        waitT(.5);
        intakeMotors.stopIntakeBall();
        stagingServos.flip();
        waitT(.25);
        intakeMotors.intakeBall(1);
        stagingServos.pushOutOfRamp();
        waitT(.25);
        intakeMotors.stopIntakeBall();
        stagingServos.flip();
        waitT(.1);

        autonShooting = false;
        intakeMotors.intakeBall(1);

    }


    public void waitT(double time){
        runtime.reset();
        while(runtime.seconds() < time) {
            //wait for b second
        }
    }

    public boolean isAutonShooting(){
        return autonShooting;
    }
    /**
     * Shoots the ball: 1 for forward, -1 for reverse
     */
    public void increaseFlywheelSpeed(){
        power = Math.min(1, power + SHOOTER_POWER_INCREMENT);
    }
    public void decreaseFlywheelSpeed(){
        power = Math.max(.6, power - SHOOTER_POWER_INCREMENT);
    }
    public void shooterPowerToggle(){
        if(toggle){
           power = CLOSE_SHOOTER_POWER;
        }else{
            power= FAR_SHOOTER_POWER;
        }
        toggle = !toggle;
    }

    public double getFlywheelSpeed(){
        return power;
    }


    public void shooterSpin(int direction) {
        outtakeMotor1.setPower(power * direction);
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