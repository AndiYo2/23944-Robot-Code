package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.ShootingSequenceConstants;
import subsystems.Spindexer;

/**
 * Extends the spindexer flipper, clears slot 1 tracking, and waits for SPINDEXER_FLICK_TIME.
 * Atomic command for pipelined shooting sequences.
 */
public class ExtendSpindexerFlipperCommand extends CommandBase {
    private final Spindexer spindexer;
    private final ElapsedTime timer = new ElapsedTime();

    public ExtendSpindexerFlipperCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.extendFlipper();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= ShootingSequenceConstants.SPINDEXER_FLICK_TIME;
    }
}
