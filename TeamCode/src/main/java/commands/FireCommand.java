package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants.FlickState;
import subsystems.Shooter;

/**
 * Triggers the shooter.
 * - Waits for shooter to be ready before triggering (handles back-to-back shots)
 * - Waits for shooter to return to idle after triggering
 */
public class FireCommand extends CommandBase {
    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean triggered = false;

    public FireCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.reset();
        triggered = false;
        if (shooter.getCurrentState() == FlickState.Idle) {
            shooter.triggerShot();
            triggered = true;
        }
    }

    @Override
    public void execute() {
        if (!triggered && shooter.getCurrentState() == FlickState.Idle) {
            shooter.triggerShot();
            triggered = true;
        }
    }

    @Override
    public boolean isFinished() {
        return (triggered && shooter.getCurrentState() == FlickState.Idle) || timer.seconds() >= 1.5;
    }

    @Override
    public void end(boolean interrupted) {
    }
}
