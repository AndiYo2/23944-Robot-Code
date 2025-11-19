package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import utility.RobotHardware;

public class ColorSensor implements Subsystem {

    NormalizedRGBA colors;
    private NormalizedColorSensor colorSensor;

    RobotHardware robot;
    public ColorSensor(){
        robot = new RobotHardware();
        refreshScan();
    }


    public void refreshScan(){
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
    public String getColorDataString(){

        return "red = [" + colors.red + "] green = [ "+ colors.green + "] blue = [ "+ colors.blue + "]";
    }
}
