package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotHardware;
import Constants.TurretConstants;

/**
 * Turret Centering Tool - Sets turret to center position (0 degrees).
 *
 * Uses direct servo position control (no PID needed).
 * Servo position 0.5 = turret center (0 degrees)
 *
 * CONTROLS:
 *   A: Toggle servo ON/OFF (release servo when off)
 *   X: Emergency release (sets servo to center and stops)
 */
@TeleOp(name = "Turret Centering Tool", group = "Tests")
public class TurretCenteringTool extends OpMode {

    private RobotHardware robot;
    private boolean servoEnabled = true;
    private boolean aButtonPressed = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        telemetry.addLine("========================================");
        telemetry.addLine("   TURRET CENTERING TOOL");
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addLine("Holds turret at 0 degrees (center)");
        telemetry.addLine("Servo position 0.5 = center");
        telemetry.addLine();
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("  A: Toggle Servo ON/OFF");
        telemetry.addLine("  X: Emergency Release");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
    }

    @Override
    public void loop() {
        PanelsConfigurables.INSTANCE.refreshClass(TurretConstants.class);

        if (gamepad1.x) {
            robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
            servoEnabled = false;
            telemetry.addLine("========================================");
            telemetry.addLine("        EMERGENCY RELEASE");
            telemetry.addLine("========================================");
            telemetry.addLine("Servo set to center and disabled");
            telemetry.addLine("Press STOP to exit");
            telemetry.update();
            return;
        }


        if (gamepad1.a && !aButtonPressed) {
            servoEnabled = !servoEnabled;
        }
        aButtonPressed = gamepad1.a;


        if (servoEnabled) {
            robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
        }


        double currentServoPos = TurretConstants.SERVO_CENTER_POSITION;
        double turretDegrees = (currentServoPos - TurretConstants.SERVO_CENTER_POSITION) *
                TurretConstants.SERVO_DEGREES_PER_UNIT / TurretConstants.GEAR_RATIO;


        telemetry.addLine("========================================");
        if (servoEnabled) {
            telemetry.addLine("   SERVO ON - HOLDING AT CENTER");
        } else {
            telemetry.addLine("   SERVO OFF - FREE TO MOVE");
        }
        telemetry.addLine("========================================");
        telemetry.addLine();
        telemetry.addData("Servo Status", servoEnabled ? "ON (holding position)" : "OFF (released)");
        telemetry.addLine();
        telemetry.addData("Target Position", "0.00 degrees (center)");
        telemetry.addData("Servo Position", "%.4f", TurretConstants.SERVO_CENTER_POSITION);
        telemetry.addData("Turret Degrees", "%.2f", turretDegrees);
        telemetry.addLine();
        telemetry.addLine("--- CONFIGURATION ---");
        telemetry.addData("Gear Ratio", "%.1f:1", TurretConstants.GEAR_RATIO);
        telemetry.addData("Hard Stop CW", "%.1f deg", TurretConstants.HARD_STOP_CW);
        telemetry.addData("Hard Stop CCW", "%.1f deg", TurretConstants.HARD_STOP_CCW);
        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("A Button", "Toggle Servo: %s", servoEnabled ? "ON->OFF" : "OFF->ON");
        telemetry.addLine("X Button: Emergency Release");
        telemetry.addLine("========================================");
        telemetry.update();
    }

    @Override
    public void stop() {
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
    }
}
