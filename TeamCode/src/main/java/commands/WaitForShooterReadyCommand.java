package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Shooter;

/**
 * Waits for shooter flywheel to reach target velocity.
 * Completes when velocity is within tolerance OR timeout expires.
 */
public class WaitForShooterReadyCommand extends CommandBase {
    private final Shooter shooter;
    private final double timeout;
    private ElapsedTime timer;

    public WaitForShooterReadyCommand(Shooter shooter, double timeoutSeconds) {
        this.shooter = shooter;
        this.timeout = timeoutSeconds;
        // Note: No addRequirements - we're just observing, not controlling
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
    }

    @Override
    public void execute() {
        // Nothing to do - just waiting lmao
    }

    @Override
    public boolean isFinished() {
        return shooter.isAtTargetVelocity() || timer.seconds() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        // Nothing to clean up
    }
}
