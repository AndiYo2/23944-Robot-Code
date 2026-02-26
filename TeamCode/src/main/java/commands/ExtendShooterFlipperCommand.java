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

    public ExtendShooterFlipperCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.extendFlipper();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= ShootingSequenceConstants.SHOOTER_FLICK_TIME;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.retractFlipper();
        }
    }
}
