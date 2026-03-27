package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotHardware;

/**
 * Waits until all 3 sensor positions report a ball present (= spindexer full).
 * Completes when all 3 balls detected OR timeout expires.
 * Uses cached quickCheck() values from round-robin polling — no I2C on command thread.
 */
public class WaitForBallsCommand extends CommandBase {
    private final double timeout;
    private ElapsedTime timer;

    public WaitForBallsCommand(double timeoutSeconds) {
        this.timeout = timeoutSeconds;
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        if (timer.seconds() >= timeout) {
            return true;
        }

        RobotHardware hw = RobotHardware.getInstance();
        boolean intakeFull = hw.spindexerSensorPair.quickCheck().ballPresent;
        boolean rampFull = hw.rampSensorPair.quickCheck().ballPresent;
        boolean transferFull = hw.transferSensorPair.quickCheck().ballPresent;

        return intakeFull && rampFull && transferFull;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
