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

    private ColorSensor colorSensor; // Declare the color sensor object

    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map
        // "color_sensor" should match the name configured in the Robot Configuration
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set the sensor's gain (optional, adjust as needed for lighting conditions)
        // A higher gain amplifies the signal, potentially making it more sensitive to subtle color differences

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double red;
        double blue;
        double green;

        waitForStart(); // Wait for the start button to be pressed

        while (opModeIsActive()) {
            // Get the normalized RGBA values from the sensor

            red = colorSensor.red();
            green = colorSensor.green();
            blue = colorSensor.blue();



            // Display the color values on the Driver Station telemetry
            telemetry.addData("Red",  red);
            telemetry.addData("Green",  green);
            telemetry.addData("Blue",  blue);
            telemetry.addData("Alpha",  colorSensor.alpha()); // Alpha represents overall brightness/intensity

            telemetry.update(); // Update the telemetry display
        }
    }
}