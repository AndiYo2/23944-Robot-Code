package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

/**
 * Triggers the spindexer flipper.
 * - Waits for flipper to be ready before triggering (handles back-to-back flicks)
 * - Waits for flipper to return to ready after triggering
 */
public class FlickCommand extends CommandBase {
    private final Spindexer spindexer;
    private boolean triggered = false;

    public FlickCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        triggered = false;
    }

    @Override
    public void execute() {
        // Wait for flipper to be ready, then trigger
        if (!triggered && spindexer.isReadyToFlip()) {
            spindexer.triggerFlick();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Finished when we've triggered AND spindexer is ready to flip again
        return triggered && spindexer.isReadyToFlip();
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed - spindexer state machine handles retraction
    }
}
