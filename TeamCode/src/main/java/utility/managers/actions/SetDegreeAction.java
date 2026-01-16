package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;

/**
 * Sets the spindexer to a specific degree position.
 */
public class SetDegreeAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final int degrees;
    private final ElapsedTime timer = new ElapsedTime();

    public SetDegreeAction(Spindexer spindexer, int degrees) {
        this.spindexer = spindexer;
        this.degrees = degrees;
    }

    @Override
    public void start() {
        spindexer.setDegree(degrees);
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
        return "SetDegree[" + degrees + "]";
    }
}
