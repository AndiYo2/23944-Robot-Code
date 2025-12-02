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
    
    public static BallPattern.BallType getBallColor(){
        refreshScan();
        float red = colors.red;
        float green = colors.green;
        float blue = colors.blue;

        // Threshold values for color detection
        final float COLOR_THRESHOLD = 0.3f;

        // Check for purple (high red and blue, low green)
        if (red > COLOR_THRESHOLD && blue > COLOR_THRESHOLD && green < COLOR_THRESHOLD) {
            return BallPattern.BallType.PURPLE;
        }
        // Check for green (high green, low red and blue)
        else if (green > COLOR_THRESHOLD && red < COLOR_THRESHOLD && blue < COLOR_THRESHOLD) {
            return BallPattern.BallType.GREEN;
        }
        // If no clear color pattern is detected
        return BallPattern.BallType.NONE;

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
