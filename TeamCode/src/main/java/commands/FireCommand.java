package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import Constants.EnumConstants.FlickState;
import subsystems.Shooter;

/**
 * Triggers the shooter.
 * - Waits for shooter to be ready before triggering (handles back-to-back shots)
 * - Waits for shooter to return to idle after triggering
 */
public class FireCommand extends CommandBase {
    private final Shooter shooter;
    private boolean triggered = false;

    public FireCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        triggered = false;
    }

    @Override
    public void execute() {
        // Wait for shooter to be ready, then trigger
        if (!triggered && shooter.getCurrentState() == FlickState.Idle) {
            shooter.triggerShot();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        // Finished when we've triggered AND shooter has returned to idle
        return triggered && shooter.getCurrentState() == FlickState.Idle;
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed - shooter state machine handles retraction
    }
}
