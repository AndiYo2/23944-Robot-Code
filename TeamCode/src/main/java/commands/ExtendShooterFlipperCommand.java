package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.ShootingSequenceConstants;
import subsystems.Shooter;

/**
 * Extends the shooter flipper and waits for SHOOTER_FLICK_TIME.
 * Atomic command for pipelined shooting sequences.
 */
public class ExtendShooterFlipperCommand extends CommandBase {
    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private final double flickTime;

    public ExtendShooterFlipperCommand(Shooter shooter) {
        this(shooter, ShootingSequenceConstants.SHOOTER_FLICK_TIME);
    }

    public ExtendShooterFlipperCommand(Shooter shooter, double flickTime) {
        this.shooter = shooter;
        this.flickTime = flickTime;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.extendFlipper();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= flickTime;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.retractFlipper();
        }
    }
}
