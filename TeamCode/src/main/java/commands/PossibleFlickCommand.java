package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

import java.util.function.BooleanSupplier;

/**
 * Conditionally triggers a flick based on a condition.
 * If condition is true, triggers the flick and runs the onFlick callback.
 * If condition is false, completes immediately.
 */
public class PossibleFlickCommand extends CommandBase {
    private final Spindexer spindexer;
    private final BooleanSupplier condition;
    private final Runnable onFlick;
    private boolean shouldFlick = false;

    public PossibleFlickCommand(Spindexer spindexer, BooleanSupplier condition, Runnable onFlick) {
        this.spindexer = spindexer;
        this.condition = condition;
        this.onFlick = onFlick;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        shouldFlick = condition.getAsBoolean();
        if (shouldFlick) {
            spindexer.triggerFlick();
            if (onFlick != null) {
                onFlick.run();
            }
        }
    }

    @Override
    public boolean isFinished() {
        // If condition was false, complete immediately
        if (!shouldFlick) {
            return true;
        }
        // Otherwise wait for spindexer to be ready to flip again
        return spindexer.isReadyToFlip();
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }
}
