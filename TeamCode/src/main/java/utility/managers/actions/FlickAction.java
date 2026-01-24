package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;

/**
 * Triggers the spindexer flipper.
 * - Waits for flipper to be ready before triggering (handles back-to-back flicks)
 * - Only waits for EXTEND time after triggering (rotation can overlap with retract)
 */
public class FlickAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean flickTriggered = false;

    public FlickAction(Spindexer spindexer) {
        this.spindexer = spindexer;
    }

    @Override
    public void start() {
        flickTriggered = false;
        timer.reset();
    }

    @Override
    public void update() {
        // Wait for flipper to be ready, then trigger
        if (!flickTriggered && spindexer.isReadyToFlip()) {
            spindexer.triggerFlick();
            flickTriggered = true;
            timer.reset();
        }
    }

    @Override
    public boolean isComplete() {
        if (!flickTriggered) {
            // Still waiting for flipper to be ready - timeout safety
            return timer.seconds() >= 0.5;
        }
        // Only wait for extend time - rotation can happen during retract
        return timer.seconds() >= SpindexerConstants.SPINDEXER_FLIPPER_OUT_TIME;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return flickTriggered ? "Flick" : "Flick[wait]";
    }
}
