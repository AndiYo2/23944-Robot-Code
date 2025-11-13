package OldSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class ColorSensor {
    NormalizedRGBA colors;
    private NormalizedColorSensor colorSensor;

    public ColorSensor(){colors = colorSensor.getNormalizedColors();}


    public void initColorSensor(HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
    }
    public void refreshScan(){colors = colorSensor.getNormalizedColors();}



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
