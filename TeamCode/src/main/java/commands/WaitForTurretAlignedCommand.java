package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Turret;

/**
 * Waits for turret to be aligned with target (not out of range).
 * Completes when turret is aligned OR timeout expires.
 */
public class WaitForTurretAlignedCommand extends CommandBase {
    private final Turret turret;
    private final double timeout;
    private ElapsedTime timer;

    public WaitForTurretAlignedCommand(Turret turret, double timeoutSeconds) {
        this.turret = turret;
        this.timeout = timeoutSeconds;
        // Note: No addRequirements - we're just observing, not controlling anything lol
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        // Nothing to do - just waiting
    }

    @Override
    public boolean isFinished() {
        return !turret.isTargetOutOfRange() || timer.seconds() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up
    }
}
