package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import utility.RobotHardware;

public class Spindexer implements Subsystem {
    RobotHardware robot;
    private static final double ROTATION_TIME_MS = 278; // Time to rotate 120 degrees
    private static final double ROTATION_POWER = 1;

    private long startTime = 0;
    private boolean isRotating = false;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
    }

    public void rotate() {
        if (!isRotating) {
            isRotating = true;
            startTime = System.currentTimeMillis();
            robot.spindexerMotor.setPower(ROTATION_POWER);
        }
    }

    @Override
    public void periodic() {
        if (isRotating) {
            if (System.currentTimeMillis() - startTime >= ROTATION_TIME_MS) {
                robot.spindexerMotor.setPower(0);
                isRotating = false;
            }
        }
    }
}
