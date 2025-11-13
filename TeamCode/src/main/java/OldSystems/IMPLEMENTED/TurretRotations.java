package OldSystems.IMPLEMENTED;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretRotations {
    double power = 1;
    private final double MAX_ROTATION_TIME = 5000; // 5 seconds in milliseconds
    private double accumulatedTime = 0;
    private int lastDirection = 0;
    private ElapsedTime runtime = new ElapsedTime();

    //servo init
    HardwareMap hMap;
    CRServo serv;


    public TurretRotations() {
    }

    public void initTurret(HardwareMap hardwareMap){
        hMap = hardwareMap;
        serv = hardwareMap.get(CRServo.class, "turretControl");

    }


    public void stop(){
        serv.setPower(0);
    }

    public void rotate(int posOrNeg){
        if (posOrNeg != lastDirection) {
            runtime.reset();
        }
        if (accumulatedTime < MAX_ROTATION_TIME || posOrNeg == 0) {
            serv.setPower(power * posOrNeg);
            if (posOrNeg != 0) {
                accumulatedTime += runtime.milliseconds();
                lastDirection = posOrNeg;
                runtime.reset();
            }
        } else {
            stop();
        }
    }




}
