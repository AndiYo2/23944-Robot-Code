package org.firstinspires.ftc.teamcode.InternalSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Timer;
import java.util.TimerTask;

/**
 * Controls the staging motors for the robot's internal systems.
 * This class manages the movement and control of staging mechanisms.
 */
public class StagingMotors {
    private static DcMotorEx stagingMotor;
    private final double speed = 1;
    private boolean running = false;

    /**
     * Initializes the staging motors with the given hardware map.
     *
     * @param hMap Hardware map containing the motor configurations
     */
    public void initStagingMotors(HardwareMap hMap) {
        stagingMotor = hMap.get(DcMotorEx.class, "stagingMotor");
        stagingMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    /**
     * Toggles the staging motor between running and stopped states.
     */
    public void runStaging(){
        stagingMotor.setPower(speed);
    }
    
    
    public void invertStagingForLaunch(){
        stagingMotor.setPower(-speed);
        try {
            Thread.sleep(20);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stopStaging();
    }
    public void invertStagingForLaunchAuto(){
        stagingMotor.setPower(-speed);
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        stopStaging();
    }

    /**
     * Stops the staging motor.
     */
    public void stopStaging() {
        stagingMotor.setPower(0);
    }
}