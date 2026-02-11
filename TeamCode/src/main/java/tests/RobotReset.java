package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import Constants.TurretConstants;
import utility.RobotHardware;

/**
 * Resets all servos to their default/safe positions.
 *
 * Servo defaults:
 *   Turret             = 0.5 (center)
 *   Spindexer          = 0.5 (center)
 *   Hood               = 1.0 (min angle / retracted)
 *   Spindexer Flipper  = retract position
 *   Shooter Flipper    = retract position
 *
 * Press START to apply reset. Telemetry confirms current positions.
 */
@TeleOp(name = "Robot Reset", group = "Tests")
public class RobotReset extends OpMode {

    private RobotHardware robot;
    private boolean resetApplied = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        telemetry.addLine("=== ROBOT RESET ===");
        telemetry.addLine("Press START to apply servo reset.");
        telemetry.addLine("All servos will move to default positions.");
        telemetry.update();
    }

    @Override
    public void start() {
        applyReset();
    }

    @Override
    public void loop() {
        // Allow re-applying reset with START button
        if (gamepad1.start) {
            applyReset();
        }

        telemetry.addLine("=== ROBOT RESET ===");
        telemetry.addData("Status", resetApplied ? "RESET APPLIED" : "WAITING");
        telemetry.addLine();

        telemetry.addLine("=== SERVO POSITIONS ===");
        telemetry.addData("Turret", "0.5 (center)");
        telemetry.addData("Spindexer", "0.5 (center)");
        telemetry.addData("Hood", "1.0 (retracted)");
        telemetry.addData("Spindexer Flipper", "%.2f (retract)", SpindexerConstants.FLIPPER_POSITION_RETRACT);
        telemetry.addData("Shooter Flipper", "%.2f (retract)", ShooterConstants.FLIPPER_POSITION_RETRACT);
        telemetry.addLine();
        telemetry.addLine("Press START to re-apply reset.");

        telemetry.update();
    }

    private void applyReset() {
        robot.turretServo.setPosition(TurretConstants.SERVO_CENTER_POSITION);
        robot.spindexerServo.setPosition(0.5);
        robot.shooterHood.setPosition(1.0);
        robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_RETRACT);
        robot.shooterFlipper.setPosition(ShooterConstants.FLIPPER_POSITION_RETRACT);
        resetApplied = true;
    }

    @Override
    public void stop() {
        // Ensure servos stay at reset positions on stop
        applyReset();
    }
}
