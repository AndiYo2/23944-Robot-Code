package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;
import utility.RobotHardware;

/**
 * Resets the spindexer to 300 degrees (empty position) and clears ball tracking.
 * Used at the end of shooting sequences to prepare for next intake cycle.
 * Also resets progressive scan so auto can re-detect balls after shooting.
 */
public class SpindexerResetCommand extends CommandBase {
    private final Spindexer spindexer;

    public SpindexerResetCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.resetToEmptyPosition();
        RobotHardware.getInstance().resetProgressiveScan();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
