package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import utility.DualBallDetector;
import Constants.NamingConstants;
import Constants.SensorConstants;


@TeleOp(name = "ColorSensorTestOpMode", group = "Tests")
public class ColorSensorTestOpMode extends LinearOpMode {

    private static final String[] PAIR_NAMES = {"INTAKE", "RAMP", "TRANSFER"};

    private ColorSensor[] sensors1 = new ColorSensor[3];
    private ColorSensor[] sensors2 = new ColorSensor[3];
    private DistanceSensor[] distSensors1 = new DistanceSensor[3];
    private DistanceSensor[] distSensors2 = new DistanceSensor[3];
    private DualBallDetector[] detectors = new DualBallDetector[3];

    private double[][] nearThresholds = {
        {SensorConstants.INTAKE_NEAR_DIST_THRESHOLD_MM},
        {SensorConstants.RAMP_NEAR_DIST_THRESHOLD_MM},
        {SensorConstants.TRANSFER_NEAR_DIST_THRESHOLD_MM}
    };
    private double[][] farThresholds = {
        {SensorConstants.INTAKE_FAR_DIST_THRESHOLD_MM},
        {SensorConstants.RAMP_FAR_DIST_THRESHOLD_MM},
        {SensorConstants.TRANSFER_FAR_DIST_THRESHOLD_MM}
    };

    private int currentPair = 0;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    @Override
    public void runOpMode() {

        sensors1[0] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.spindexerSensor1);
        sensors2[0] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.spindexerSensor2);

        sensors1[1] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor1);
        sensors2[1] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.rampSensor2);

        sensors1[2] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor1);
        sensors2[2] = hardwareMap.get(ColorSensor.class, NamingConstants.ColorSensor.transferSensor2);

        for (int i = 0; i < 3; i++) {
            distSensors1[i] = (DistanceSensor) sensors1[i];
            distSensors2[i] = (DistanceSensor) sensors2[i];
        }

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        detectors[0] = new DualBallDetector(sensors1[0], sensors2[0]); // Intake - defaults
        detectors[1] = new DualBallDetector(sensors1[1], sensors2[1]); // Ramp - defaults

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

            if (gamepad1.dpad_right && !prevDpadRight) {
                currentPair = (currentPair + 1) % 3;
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                currentPair = (currentPair + 2) % 3;
            }
            prevDpadRight = gamepad1.dpad_right;
            prevDpadLeft = gamepad1.dpad_left;

            for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
                hub.clearBulkCache();
            }

            for (DualBallDetector detector : detectors) {
                detector.update();
            }

            ColorSensor sensor1 = sensors1[currentPair];
            ColorSensor sensor2 = sensors2[currentPair];
            DistanceSensor dist1 = distSensors1[currentPair];
            DistanceSensor dist2 = distSensors2[currentPair];
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

            telemetry.addLine("--- DISTANCE DATA ---");
            double d1 = dist1.getDistance(DistanceUnit.MM);
            double d2 = dist2.getDistance(DistanceUnit.MM);
            telemetry.addData("S1 Dist (mm)", "%.1f", d1);
            telemetry.addData("S2 Dist (mm)", "%.1f", d2);
            telemetry.addData("Ball Detected (dist)",
                    d1 < nearThresholds[currentPair][0] || d2 < farThresholds[currentPair][0]);

            telemetry.addLine();

            telemetry.addLine("--- BALL DETECTION ---");
            telemetry.addData("Ball Present", result.ballPresent);
            telemetry.addData("Ball Color", result.color);
            telemetry.addData("Confidence", "%.1f%%", result.confidence * 100);

            telemetry.update();
        }
    }
}
