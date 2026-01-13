package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import utility.RobotHardware;

/**
 * Spindexer Test - Simple test for spindexer servo.
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
