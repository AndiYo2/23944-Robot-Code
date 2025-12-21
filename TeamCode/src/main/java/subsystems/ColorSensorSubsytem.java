package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;
import utility.RobotConstants.Enums.ColorSensorState;
import utility.RobotHardware;

public class ColorSensorSubsytem implements Subsystem {

    static NormalizedRGBA colors;
    private static NormalizedColorSensor colorSensor;
    static double red, green, blue, alpha;
    private static BallColor lastDetectedColor = BallColor.None;
    private ColorSensorState currentState = ColorSensorState.Scanning;
    private BallColor lastStableColor = BallColor.None;
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
        if(red > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[0] &&
           green > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[1] &&
           blue > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[2])
            return BallColor.Purple;
        else if(red > RobotConstants.ColorSensor.GREEN_THRESHOLDS[0] &&
                green < RobotConstants.ColorSensor.GREEN_THRESHOLDS[0] &&
                blue < RobotConstants.ColorSensor.GREEN_THRESHOLDS[0])
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

    private void stateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                // Sensor not actively used
                break;
            case Scanning:
                // Already refreshing in periodic()
                if (ballJustEntered()) {
                    currentState = ColorSensorState.BallDetected;
                }
                break;
            case BallDetected:
                // Transition to BallHeld after one cycle
                currentState = ColorSensorState.BallHeld;
                lastStableColor = getBallColor();
                break;
            case BallHeld:
                // Check if ball is still present
                BallColor current = getBallColor();
                if (current == BallColor.None && lastStableColor != BallColor.None) {
                    // Ball removed
                    currentState = ColorSensorState.Scanning;
                    lastStableColor = BallColor.None;
                }
                break;
        }
    }

    // ****** STATE CONTROL ******
    public void startScanning() {
        currentState = ColorSensorState.Scanning;
    }

    public void stopScanning() {
        currentState = ColorSensorState.Idle;
    }

    public boolean ballJustDetected() {
        return currentState == ColorSensorState.BallDetected;
    }

    // ****** STATE QUERIES ******
    public ColorSensorState getCurrentState() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == ColorSensorState.Idle;
    }

    @Override
    public void periodic() {
        refreshScan();
        stateMachinePeriodic();
    }

}
