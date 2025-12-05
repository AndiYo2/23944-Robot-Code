package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import utility.BallPattern;
import utility.RobotHardware;

public class ColorSensorSubsytem implements Subsystem {

    static NormalizedRGBA colors;
    private static NormalizedColorSensor colorSensor;

    RobotHardware robot;
    public ColorSensorSubsytem(){
        this.robot = RobotHardware.getInstance();
        colorSensor = (NormalizedColorSensor) robot.colorSensorIntake;
        refreshScan();
    }


    public static void refreshScan(){
        colors = colorSensor.getNormalizedColors();}



    public float getRed(){
        refreshScan();
        return colors.red;
    }
    public float getGreen(){
        refreshScan();
        return colors.green;
    }
    public float getBlue(){
        refreshScan();
        return colors.blue;
    }

    public float getAlpha(){
        refreshScan();
        return colors.alpha;
    }
    
    public static BallPattern.BallType getBallColor(){
        refreshScan();

        if(colors.alpha < .07){
            return BallPattern.BallType.PURPLE;
        }else{
            return BallPattern.BallType.NONE;
        }
    }

    public String getColorDataString(){
        refreshScan();
        return "red = [" + colors.red + "] green = [ "+ colors.green + "] blue = [ "+ colors.blue + "]";
    }
    @Override
    public void periodic() {
        refreshScan();
    }

}
