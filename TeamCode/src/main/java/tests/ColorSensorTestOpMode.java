package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor; // For getLightDetected() if needed

@TeleOp(name = "ColorSensorTestOpMode", group = "Sensor")
public class ColorSensorTestOpMode extends LinearOpMode {

    private NormalizedColorSensor colorSensor; // Declare the color sensor object

    @Override
    public void runOpMode() {
        // Initialize the color sensor from the hardware map
        // "color_sensor" should match the name configured in the Robot Configuration
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        // Set the sensor's gain (optional, adjust as needed for lighting conditions)
        // A higher gain amplifies the signal, potentially making it more sensitive to subtle color differences
        colorSensor.setGain(7); // Example gain setting

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double red;
        double blue;
        double green;

        waitForStart(); // Wait for the start button to be pressed

        while (opModeIsActive()) {
            // Get the normalized RGBA values from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            red = colors.red/ colors.alpha;
            green = colors.green/ colors.alpha;
            blue = colors.blue/ colors.alpha;



            // Display the color values on the Driver Station telemetry
            telemetry.addData("Red",  red);
            telemetry.addData("Green",  green);
            telemetry.addData("Blue",  blue);
            telemetry.addData("Alpha",  colors.alpha); // Alpha represents overall brightness/intensity


            telemetry.update(); // Update the telemetry display
        }
    }
}