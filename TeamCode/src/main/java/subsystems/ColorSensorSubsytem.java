package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import utility.RobotConstants.Enums.BallColor;
import utility.RobotHardware;

public class ColorSensorSubsytem implements Subsystem {

    static NormalizedRGBA colors;
    private static NormalizedColorSensor colorSensor;
    static double red, green, blue, alpha;
    static final double[] RED = {0.6, 0.5, 0.4};
    static final double[] GREEN = {0.3, 0.5, 0.2};
    private static BallColor lastDetectedColor = BallColor.None;
    RobotHardware robot;


    public ColorSensorSubsytem(){
        this.robot = RobotHardware.getInstance();
        colorSensor = (NormalizedColorSensor) robot.colorSensor;
        refreshScan();
    }


    public static void refreshScan(){
        colors = colorSensor.getNormalizedColors();
        red = colors.red / colors.alpha;
        green = colors.green / colors.alpha;
        blue = colors.blue / colors.alpha;
        alpha = colors.alpha;

    }
    public double getRed(){
        return red;
    }
    public double getGreen(){
        return green;
    }
    public double getBlue(){
        return colors.blue;
    }
    public double getAlpha(){
        return colors.alpha;
    }
    public static BallColor getBallColor(){
        if(red > RED[0] && green > RED[1] && blue > RED[2])
            return BallColor.Purple;
        else if(red > GREEN[0] && green < GREEN[0] && blue < GREEN[0])
            return BallColor.Green;
        return BallColor.None;
    }


    public static boolean ballJustEntered() {
        BallColor current = getBallColor();
        boolean entered = (lastDetectedColor == BallColor.None && current != BallColor.None);
        lastDetectedColor = current;
        return entered;
    }
    public String getColorDataString(){
        return "red = [" + red + "] green = [ "+ green + "] blue = [ "+ blue + "]";
    }
    @Override
    public void periodic() {
        refreshScan();
    }

}
