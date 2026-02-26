package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;

/**
 * Triggers the spindexer flipper.
 * - Waits for flipper to be ready before triggering (handles back-to-back flicks)
 * - Waits for flipper to return to ready after triggering
 */
public class FlickCommand extends CommandBase {
    private final Spindexer spindexer;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean triggered = false;

    public FlickCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        timer.reset();
        triggered = false;
        if (spindexer.isReadyToFlip()) {
            spindexer.triggerFlick();
            triggered = true;
        }
    }

    @Override
    public void execute() {
        if (!triggered && spindexer.isReadyToFlip()) {
            spindexer.triggerFlick();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (triggered && spindexer.isReadyToFlip()) || timer.seconds() >= 1.5;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
