package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.ColorSensor;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;
import utility.RobotConstants.Enums.ColorSensorState;
import utility.RobotHardware;

public class ColorSensorSubsytem implements Subsystem {

    private static ColorSensor colorSensor;
    static double red, green, blue, alpha;
    private static BallColor lastDetectedColor = BallColor.None;
    private ColorSensorState currentState = ColorSensorState.Scanning;
    private BallColor lastStableColor = BallColor.None;
    RobotHardware robot;


    public ColorSensorSubsytem(){
        this.robot = RobotHardware.getInstance();
        // For backward compatibility - uses the first intake sensor
        colorSensor = robot.intakeSensor1;
        refreshScan();
    }


    public static void refreshScan(){
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();
        alpha = colorSensor.alpha();
    }
    public double getRed(){
        return red;
    }
    public double getGreen(){
        return green;
    }
    public double getBlue(){
        return blue;
    }
    public double getAlpha(){
        return alpha;
    }
    public static BallColor getBallColor(){
        // Purple ball: High red, medium green, high blue
        if(red > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[0] &&
           green > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[1] &&
           blue > RobotConstants.ColorSensor.PURPLE_THRESHOLDS[2])
            return BallColor.Purple;
        // Green ball: Low red, high green, low blue
        else if(red < RobotConstants.ColorSensor.GREEN_THRESHOLDS[0] &&
                green > RobotConstants.ColorSensor.GREEN_THRESHOLDS[1] &&
                blue < RobotConstants.ColorSensor.GREEN_THRESHOLDS[2])
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
