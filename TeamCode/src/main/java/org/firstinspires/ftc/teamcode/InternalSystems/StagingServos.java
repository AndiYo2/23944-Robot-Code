package org.firstinspires.ftc.teamcode.InternalSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class StagingServos {

    double power = 1;
    boolean toggleStatus = false;

    HardwareMap hMap;
    CRServo leftServo, rightServo;

    public void initStagingServos(HardwareMap hardwareMap){
        hMap = hardwareMap;
        leftServo = hMap.get(CRServo.class, "leftServo");
        rightServo = hMap.get(CRServo.class, "rightServo");
        rightServo.setDirection(CRServo.Direction.REVERSE);
    }


    public void toggleStageServos(int direction){
        if(toggleStatus){
            leftServo.setPower(0);
            rightServo.setPower(0);
        }else {
            leftServo.setPower(power * direction);
            rightServo.setPower(power * direction);
        }

        toggleStatus = !toggleStatus;

    }



}
