package tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; // For getLightDetected() if needed
import utility.RobotConstants;


@TeleOp(name = "ColorSensorTestOpMode", group = "Tests")
public class ColorSensorTestOpMode extends LinearOpMode {

    private ColorSensor colorSensor1; // Declare the color sensor object
    private ColorSensor colorSensor2; // Declare the color sensor object

    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map
        // "color_sensor" should match the name configured in the Robot Configuration
        colorSensor1 = hardwareMap.get(ColorSensor.class, "intakeSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "intakeSensor2");

        // Set the sensor's gain (optional, adjust as needed for lighting conditions)
        // A higher gain amplifies the signal, potentially making it more sensitive to subtle color differences

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double red1;
        double blue1;
        double green1;
        double red2;
        double blue2;
        double green2;

        waitForStart(); // Wait for the start button to be pressed

        while (opModeIsActive()) {
            // Get the normalized RGBA values from the sensor

            red1 = colorSensor1.red();
            green1 = colorSensor1.green();
            blue1 = colorSensor1.blue();
            red2 = colorSensor1.red();
            green2 = colorSensor1.green();
            blue2 = colorSensor1.blue();



            // Display the color values on the Driver Station telemetry
            telemetry.addData("Red1",  red1);
            telemetry.addData("Green1",  green1);
            telemetry.addData("Blue1",  blue1);
            telemetry.addData("Alpha1",  colorSensor1.alpha()); // Alpha represents overall brightness/intensity
            telemetry.addData("Red2",  red2);
            telemetry.addData("Green2",  green2);
            telemetry.addData("Blue2",  blue2);
            telemetry.addData("Alpha2",  colorSensor2.alpha()); // Alpha represents overall brightness/intensity

            telemetry.update(); // Update the telemetry display
        }
    }
}