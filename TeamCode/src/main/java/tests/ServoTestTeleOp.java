package tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import Constants.NamingConstants;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        ServoImplEx spindexerServo = hardwareMap.get(ServoImplEx.class, NamingConstants.Spindexer.spindexerServo);
        spindexerServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        Servo spindexerFlipper = hardwareMap.get(Servo.class, NamingConstants.Spindexer.spindexerFlipperServo);
        Servo shooterFlipper = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterFlipperServo);
        Servo shooterHood = hardwareMap.get(Servo.class, NamingConstants.Shooter.shooterHood);
        Servo turret = hardwareMap.get(Servo.class, NamingConstants.Turret.turret);

        AnalogInput spindexerFlipperEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Spindexer.spindexerFlipperEncoder);
        AnalogInput shooterFlipperEncoder = hardwareMap.get(AnalogInput.class, NamingConstants.Shooter.shooterFlipperEncoder);

        AnalogInput[] encoders = {
                null,                       // Spindexer
                spindexerFlipperEncoder,    // Spindexer Flipper
                shooterFlipperEncoder,      // Shooter Flipper
                null,                       // Shooter Hood
                null                        // Turret
        };

        String[] names = {
                "Spindexer",
                "Spindexer Flipper",
                "Shooter Flipper",
                "Shooter Hood",
                "Turret"
        };

        Servo[] servos = {
                spindexerServo,
                spindexerFlipper,
                shooterFlipper,
                shooterHood,
                turret
        };

        double[] positions = new double[servos.length];
        for (int i = 0; i < positions.length; i++) {
            positions[i] = 0.5;
            servos[i].setPosition(0.5);
        }

        int selectedIndex = 0;

        boolean prevLeftBumper = false;
        boolean prevRightBumper = false;
        boolean prevA = false;
        boolean prevB = false;

        telemetry.addLine("Servo Test Ready");
        telemetry.addLine("Bumpers = cycle | Left Stick Y = position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean leftBumper = gamepad1.left_bumper;
            boolean rightBumper = gamepad1.right_bumper;
            boolean aButton = gamepad1.a;
            boolean bButton = gamepad1.b;

            if (rightBumper && !prevRightBumper) {
                selectedIndex = (selectedIndex + 1) % servos.length;
            }
            if (leftBumper && !prevLeftBumper) {
                selectedIndex = (selectedIndex - 1 + servos.length) % servos.length;
            }
            if (aButton && !prevA) {
                positions[selectedIndex] = 0.5;
            }

            prevLeftBumper = leftBumper;
            prevRightBumper = rightBumper;
            prevA = aButton;
            prevB = bButton;

            double stickY = -gamepad1.left_stick_y;
            if (Math.abs(stickY) > 0.05) {
                positions[selectedIndex] += stickY * 0.005;
                positions[selectedIndex] = Math.max(0.0, Math.min(1.0, positions[selectedIndex]));
            }

            servos[selectedIndex].setPosition(positions[selectedIndex]);

            telemetry.addLine("=== SERVO TEST ===");
            telemetry.addLine("");
            for (int i = 0; i < servos.length; i++) {
                String prefix = (i == selectedIndex) ? ">> " : "   ";
                if (encoders[i] != null) {
                    double actualPos = encoders[i].getVoltage() / 3.3;
                    telemetry.addData(prefix + names[i], "Cmd: %.3f | Actual: %.3f", positions[i], actualPos);
                } else {
                    telemetry.addData(prefix + names[i], "%.3f", positions[i]);
                }
            }
            telemetry.addLine("");
            telemetry.addData("Controls", "LB/RB = cycle | Stick Y = move | A = center");
            telemetry.update();
        }
    }
}