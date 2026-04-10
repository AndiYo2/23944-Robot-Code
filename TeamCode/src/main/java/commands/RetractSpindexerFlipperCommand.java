package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.ShootingSequenceConstants;
import subsystems.Spindexer;

/**
 * Retracts the spindexer flipper and waits for SPINDEXER_RETRACT_DELAY.
 * Atomic command for pipelined shooting sequences.
 */
public class RetractSpindexerFlipperCommand extends CommandBase {
    private final Spindexer spindexer;
    private final ElapsedTime timer = new ElapsedTime();

    public RetractSpindexerFlipperCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.retractFlipper();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= ShootingSequenceConstants.SPINDEXER_RETRACT_DELAY;
    }
}
