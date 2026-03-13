package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

/**
 * Rotates the spindexer counter-clockwise by one slot.
 * Completes when rotation is finished.
 */
public class RotateCCWCommand extends CommandBase {
    private final Spindexer spindexer;

    public RotateCCWCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.rotateCCW();
    }

    @Override
    public boolean isFinished() {
        return spindexer.isRotationIdle() || spindexer.isAtTargetPosition();
    }

    @Override
    public void end(boolean interrupted) {
    }
}
