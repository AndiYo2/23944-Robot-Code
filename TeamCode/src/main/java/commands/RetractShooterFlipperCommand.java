package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.ShootingSequenceConstants;
import subsystems.Shooter;

/**
 * Retracts the shooter flipper and waits for SHOOTER_RETRACT_DELAY.
 * Atomic command for pipelined shooting sequences.
 */
public class RetractShooterFlipperCommand extends CommandBase {
    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();

    public RetractShooterFlipperCommand(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.retractFlipper();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= ShootingSequenceConstants.SHOOTER_RETRACT_DELAY;
    }
}
