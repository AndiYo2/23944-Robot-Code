package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import utility.RobotHardware;
import Constants.TurretConstants;

/**
 * Turret Position Testing OpMode - Test turret at different positions.
 *
 * Uses direct servo position control (no PID needed).
 * Adjust TURRET_TUNING_TARGET in Panels dashboard to set target angle.
 *
 * Useful for:
 * - Testing hard stop limits
 * - Verifying servo range
 * - Checking turret movement
 *
 * Safety controls:
 * - X: Emergency release (return to center)
 * - A: Reset to center (0 degrees)
 */
@TeleOp(name = "TurretPositionTest", group = "Tests")
public class TurretPIDFTuningTeleOp extends OpMode {

    private RobotHardware robot;
    private boolean emergencyStop = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized - Adjust values in Panels dashboard");
        telemetry.addData("Controls", "X=Emergency Release, A=Reset to Center");
        telemetry.addLine("Adjust TURRET_TUNING_TARGET in dashboard");
        telemetry.update();
    }

    @Override
    public void start() {
        emergencyStop = false;
        // Start at center
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
    }

    @Override
    public void loop() {
        // Refresh values from Panels dashboard
        PanelsConfigurables.INSTANCE.refreshClass(TurretConstants.class);

        // EMERGENCY RELEASE - X button
        if (gamepad1.x) {
            emergencyStop = true;
            robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
        }

        // Reset from emergency - A button
        if (gamepad1.a) {
            emergencyStop = false;
            TurretConstants.TURRET_TUNING_TARGET = 0;
        }

        if (emergencyStop) {
            robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
            telemetry.addLine("====== EMERGENCY RELEASE ======");
            telemetry.addLine("Servo at center position");
            telemetry.addLine("Press A to reset and continue");
            telemetry.update();
            return;
        }

        // Get target from TurretConstants (turret degrees)
        double targetTurretDegrees = TurretConstants.TURRET_TUNING_TARGET;

        // Clamp to hard stop limits
        targetTurretDegrees = Math.max(TurretConstants.HARD_STOP_CCW,
                                       Math.min(TurretConstants.HARD_STOP_CW, targetTurretDegrees));

        // Convert to servo position
        double servoPosition = TurretConstants.SERVO_CENTER_POSITION +
                (targetTurretDegrees * TurretConstants.GEAR_RATIO / TurretConstants.SERVO_DEGREES_PER_UNIT);

        // Clamp servo position and ensure never exactly 0
        servoPosition = Math.max(TurretConstants.MIN_SERVO_POSITION, Math.min(1.0, servoPosition));

        // Apply servo position
        robot.turretServo.setPosition(servoPosition);

        // Check if near limits
        boolean nearCWLimit = targetTurretDegrees > (TurretConstants.HARD_STOP_CW - 5);
        boolean nearCCWLimit = targetTurretDegrees < (TurretConstants.HARD_STOP_CCW + 5);

        // Telemetry - Safety warnings first
        if (nearCWLimit) {
            telemetry.addLine("WARNING: Near CW limit!");
        } else if (nearCCWLimit) {
            telemetry.addLine("WARNING: Near CCW limit!");
        }

        telemetry.addLine("====== TURRET POSITION TEST ======");
        telemetry.addData("Target (Turret)", "%.1f deg", targetTurretDegrees);
        telemetry.addData("Servo Position", "%.4f", servoPosition);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("--- LIMITS ---");
        telemetry.addData("Hard Stop CW", "%.1f deg", TurretConstants.HARD_STOP_CW);
        telemetry.addData("Hard Stop CCW", "%.1f deg", TurretConstants.HARD_STOP_CCW);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("--- CONFIGURATION ---");
        telemetry.addData("Gear Ratio", "%.1f:1", TurretConstants.GEAR_RATIO);
        telemetry.addData("Servo Center", "%.4f", TurretConstants.SERVO_CENTER_POSITION);
        telemetry.addData("Min Change Threshold", "%.1f deg", TurretConstants.MIN_CHANGE_THRESHOLD);
        telemetry.addLine("-----------------------------");
        telemetry.addLine("Adjust TURRET_TUNING_TARGET in Panels");
        telemetry.addLine("X=Emergency Release, A=Reset to Center");
        telemetry.update();
    }

    @Override
    public void stop() {
        // Return to center on stop
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
    }
}
