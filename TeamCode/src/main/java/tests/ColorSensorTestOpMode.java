package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import utility.DualBallDetector;


@TeleOp(name = "ColorSensorTestOpMode", group = "Tests")
public class ColorSensorTestOpMode extends LinearOpMode {

    private ColorSensor colorSensor1;
    private ColorSensor colorSensor2;

    private DualBallDetector ballDetector;

    @Override
    public void runOpMode() {

        colorSensor1 = hardwareMap.get(ColorSensor.class, "intakeSensor1");
        colorSensor2 = hardwareMap.get(ColorSensor.class, "intakeSensor2");

        ballDetector = new DualBallDetector(
                colorSensor1,
                colorSensor2
        );

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === REQUIRED EVERY LOOP ===
            ballDetector.update();

            // === Get stabilized result ===
            DualBallDetector.Result result = ballDetector.detectBall();

            // ============================
            // Raw Sensor Telemetry
            // ============================
            telemetry.addLine("=== RAW SENSOR DATA ===");

            telemetry.addData("S1 R", colorSensor1.red());
            telemetry.addData("S1 G", colorSensor1.green());
            telemetry.addData("S1 B", colorSensor1.blue());
            telemetry.addData("S1 A", colorSensor1.alpha());

            telemetry.addLine();

            telemetry.addData("S2 R", colorSensor2.red());
            telemetry.addData("S2 G", colorSensor2.green());
            telemetry.addData("S2 B", colorSensor2.blue());
            telemetry.addData("S2 A", colorSensor2.alpha());

            telemetry.addLine();

            // ============================
            // Processed Detection Telemetry
            // ============================
            telemetry.addLine("=== BALL DETECTION ===");
            telemetry.addData("Ball Present", result.ballPresent);
            telemetry.addData("Ball Color", result.color);
            telemetry.addData("Confidence", "%.1f%%", result.confidence * 100);

            telemetry.update();
        }
    }
}
