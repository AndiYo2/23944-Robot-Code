package tests;

import com.bylazar.configurables.PanelsConfigurables;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.TurretConstants;

/**
 * Turret Centering Tool - Simplified version using Panels dashboard.
 *
 * This OpMode holds the turret at exactly 0° (center position).
 * Useful for verifying encoder calibration or testing turret control.
 *
 * Adjust PID values in Panels dashboard:
 * - TurretConstants.TURRET_PID (PIDCoefficients object with p, i, d)
 *
 * CONTROLS:
 *   A: Toggle Motor ON/OFF
 *   X: Emergency Stop
 */
@TeleOp(name = "Spindexer0", group = "Tests")
public class Spindexer0 extends OpMode {

    private RobotHardware robot;

    // PID variables
    private double targetPosition = 1.0;  // Always 0° (center)


    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);
    }

    @Override
    public void start() {
        robot.spindexerServo.setPosition(targetPosition);
    }

    @Override
    public void loop() {
        robot.spindexerServo.setPosition(1);
    }


    @Override
    public void stop() {
    }
}
