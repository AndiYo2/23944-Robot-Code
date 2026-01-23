package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import subsystems.Spindexer;

/**
 * Rotates the spindexer to the next closest ball.
 * Completes when rotation is finished.
 */
public class RotateToNextCommand extends CommandBase {
    private final Spindexer spindexer;

    public RotateToNextCommand(Spindexer spindexer) {
        this.spindexer = spindexer;
        addRequirements(spindexer);
    }

    @Override
    public void initialize() {
        spindexer.rotateToNextClosestBall();
    }

    @Override
    public boolean isFinished() {
        return spindexer.isRotationIdle();
    }

    @Override
    public void end(boolean interrupted) {
        // No cleanup needed
    }
}
