package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

/**
 * Rotates the spindexer clockwise by one slot.
 * Completes when rotation is finished.
 */
public class RotateCWCommand extends CommandBase {
    private final Spindexer spindexer;

    public RotateCWCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.rotateCW();
    }

    @Override
    public boolean isFinished() {
        return spindexer.isRotationIdle() || spindexer.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
