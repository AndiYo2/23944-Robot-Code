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

        waitForStart(); // Wait for the start button to be pressed

        while (opModeIsActive()) {
            // Get the normalized RGBA values from the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Display the color values on the Driver Station telemetry
            telemetry.addData("Red", "%.3f", colors.red);
            telemetry.addData("Green", "%.3f", colors.green);
            telemetry.addData("Blue", "%.3f", colors.blue);
            telemetry.addData("Alpha", "%.3f", colors.alpha); // Alpha represents overall brightness/intensity

            // Example of using light detected (if the sensor also functions as an OpticalDistanceSensor)
            if (colorSensor instanceof OpticalDistanceSensor) {
                telemetry.addData("Light Detected", ((OpticalDistanceSensor) colorSensor).getLightDetected());
            }

            // Add logic here to react to specific colors, e.g.,
            // if (colors.red > colors.blue && colors.red > colors.green && colors.red > 0.5) {
            //     telemetry.addData("Detected", "Red");
            // }

            telemetry.update(); // Update the telemetry display
        }
    }
}