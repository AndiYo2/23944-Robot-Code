package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;

/**
 * Rotates the spindexer counter-clockwise by one position (60°).
 * Updates ball pattern tracking automatically.
 */
public class RotateCCWAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final ElapsedTime timer = new ElapsedTime();

    public RotateCCWAction(Spindexer spindexer) {
        this.spindexer = spindexer;
    }

    @Override
    public void start() {
        spindexer.rotateCCW();
        timer.reset();
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return timer.seconds() >= SpindexerConstants.ROTATION_TIME;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return "RotateCCW";
    }
}
