package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

/**
 * Resets the spindexer to 300 degrees (empty position) and clears ball tracking.
 * Used at the end of shooting sequences to prepare for next intake cycle.
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
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
