package org.firstinspires.ftc.teamcode.InternalSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StagingServos {

    double power = 1;
    boolean toggleStatus = false;

    HardwareMap hMap;
    Servo flipperServo, rampServo;
    ElapsedTime runtime = new ElapsedTime();

    public void initStagingServos(HardwareMap hardwareMap){
        hMap = hardwareMap;
        flipperServo = hMap.get(Servo.class,"flipperServo");
        rampServo = hMap.get(Servo.class, "rampServo");
    }
    
    public boolean flip(){
        runtime.reset();
        flipperServo.setPosition(0.5);
        while (runtime.seconds() < 0.25) {
            // Wait for 0.25 seconds
        }
        flipperServo.setPosition(0.0);
        return true;
    }

    public boolean pushOutOfRamp() {
        runtime.reset();
        rampServo.setPosition(0.25);
        while(runtime.seconds() < 0.25) {
            //Wait for 0.25 seconds
        }
        rampServo.setPosition(0.0);
        return true;
    }


   /* public void toggleStageServos(int direction){
        if(toggleStatus){
            leftServo.setPower(0);
            rightServo.setPower(0);
        }else {
            leftServo.setPower(power * direction);
            rightServo.setPower(power * direction);
        }

        toggleStatus = !toggleStatus;

    }*/



}
