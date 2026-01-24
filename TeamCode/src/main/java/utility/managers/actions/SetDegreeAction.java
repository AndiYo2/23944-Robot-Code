package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;
import utility.managers.SpindexerManager;

/**
 * Sets the spindexer to a specific degree position.
 */
public class SetDegreeAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final int degrees;
    private final ElapsedTime timer = new ElapsedTime();
    private final boolean resetCalcs;

    public SetDegreeAction(Spindexer spindexer, int degrees) {
        this.spindexer = spindexer;
        this.degrees = degrees;
        this.resetCalcs = false;
    }
    public SetDegreeAction(Spindexer spindexer, int degrees, boolean resetCalcs) {
        this.spindexer = spindexer;
        this.degrees = degrees;
        this.resetCalcs = resetCalcs;
    }

    @Override
    public void start() {
        spindexer.setDegree(degrees);
        timer.reset();
        if(resetCalcs){
        }
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
