package org.firstinspires.ftc.teamcode.Outtake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Globals.AxonServo;

public class TurretRotations {
    //Might be 44 for something idk
    static double position = 0;
    double power = 1;
    //Position is the same as the degree of the turret in our arc
    final double negBound = -360*3, posBound = 360*3;

    //servo init
    HardwareMap hMap;
    CRServo serv;


    public TurretRotations() {
    }

    public void initTurret(HardwareMap hardwareMap){
        hMap = hardwareMap;
        serv = hardwareMap.get(CRServo.class, "turretControl");

    }

    /*public boolean rotateToX(double newPos){
        if (newPos < posBound && newPos > negBound){
            position = newPos;
            if(newPos > position){
                serv.setDirection(CRServo.Direction.FORWARD);
            }else{
                serv.setDirection(CRServo.Direction.REVERSE);
            }
            serv.setTargetRotation(newPos);
            return true;
        }
        return false;
    }*/

    public void stop(){
        serv.setPower(0);
    }

    public void rotate(int posOrNeg){
        serv.setPower(power * posOrNeg);

    }

    /*public boolean checkViewForTag(){
        if(true)//is the thing in view
        {
            rotateToX( -1);
            return true;
        }
        return false;
    }

    public void rotateBot(){
        //Temp
    }

    public void setToTag(){
        rotateToX(negBound + 1);
        while(!checkViewForTag()){
            if(!rotateToX(position + 1))
                rotateBot();
            break;

        }
        //lockOnToTag()


    }*/


}
