package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import utility.DualBallDetector;
import Constants.NamingConstants;


@TeleOp(name = "ColorSensorTestOpMode", group = "Tests")
public class ColorSensorTestOpMode extends LinearOpMode {

    private static final String[] PAIR_NAMES = {"INTAKE", "RAMP", "TRANSFER"};

    private ColorSensor[] sensors1 = new ColorSensor[3];
    private ColorSensor[] sensors2 = new ColorSensor[3];
    private DualBallDetector[] detectors = new DualBallDetector[3];

    private int currentPair = 0;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {

        sensors1[0] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor1);
        sensors2[0] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.intakeSensor2);

        sensors1[1] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor1);
        sensors2[1] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor2);

        sensors1[2] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor1);
        sensors2[2] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor2);

        detectors[0] = new DualBallDetector(sensors1[0], sensors2[0]); // Intake - defaults
        detectors[1] = new DualBallDetector(sensors1[1], sensors2[1]); // Ramp - defaults

        // Transfer - custom profiles and thresholds (must match RobotHardware)
        double[] transferGreen = {0.21, 0.47, 0.32};
        double[] transferPurple = {0.32, 0.31, 0.37};
        detectors[2] = new DualBallDetector(sensors1[2], sensors2[2],
                transferGreen, transferPurple, 0.15, 0.15,
                200, 400);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "DPAD Left/Right to cycle pairs");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // === Handle pair cycling with edge detection ===
            if (gamepad1.dpad_right && !prevDpadRight) {
                currentPair = (currentPair + 1) % 3;
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                currentPair = (currentPair + 2) % 3;
            }
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;

            for (DualBallDetector detector : detectors) {
                detector.update();
            }

            ColorSensor sensor1 = sensors1[currentPair];
            ColorSensor sensor2 = sensors2[currentPair];
            DualBallDetector.Result result = detectors[currentPair].detectBall();


            telemetry.addLine("=== " + PAIR_NAMES[currentPair] + " SENSORS ===");
            telemetry.addData("DPAD L/R", "Cycle pairs");
            telemetry.addLine();

            telemetry.addLine("--- RAW SENSOR DATA ---");

            telemetry.addData("S1 R", sensor1.red());
            telemetry.addData("S1 G", sensor1.green());
            telemetry.addData("S1 B", sensor1.blue());
            telemetry.addData("S1 A", sensor1.alpha());

            telemetry.addLine();

            telemetry.addData("S2 R", sensor2.red());
            telemetry.addData("S2 G", sensor2.green());
            telemetry.addData("S2 B", sensor2.blue());
            telemetry.addData("S2 A", sensor2.alpha());

            telemetry.addLine();


            telemetry.addLine("--- NORMALIZED RGB ---");
            double sum1 = sensor1.red() + sensor1.green() + sensor1.blue();
            double sum2 = sensor2.red() + sensor2.green() + sensor2.blue();

            if (sum1 > 0) {
                telemetry.addData("S1 Norm", "R:%.2f G:%.2f B:%.2f",
                        sensor1.red() / sum1, sensor1.green() / sum1, sensor1.blue() / sum1);
            } else {
                telemetry.addData("S1 Norm", "N/A");
            }

            if (sum2 > 0) {
                telemetry.addData("S2 Norm", "R:%.2f G:%.2f B:%.2f",
                        sensor2.red() / sum2, sensor2.green() / sum2, sensor2.blue() / sum2);
            } else {
                telemetry.addData("S2 Norm", "N/A");
            }

            telemetry.addLine();

            telemetry.addLine("--- BALL DETECTION ---");
            telemetry.addData("Ball Present", result.ballPresent);
            telemetry.addData("Ball Color", result.color);
            telemetry.addData("Confidence", "%.1f%%", result.confidence * 100);

            telemetry.update();
        }
    }
}
