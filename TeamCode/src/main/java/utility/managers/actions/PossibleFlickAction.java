package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;

import java.util.function.BooleanSupplier;

/**
 * Conditionally flicks a ball to the shooter.
 * - Checks condition at start to decide whether to flick
 * - If condition is true, flicks and waits for extend time
 * - If condition is false, completes immediately
 * - Can be run in parallel with intake
 */
public class PossibleFlickAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final BooleanSupplier shouldFlick;
    private final Runnable onFlick;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean flickTriggered = false;
    private boolean shouldFlickResult = false;

    public PossibleFlickAction(Spindexer spindexer, BooleanSupplier shouldFlick, Runnable onFlick) {
        this.spindexer = spindexer;
        this.shouldFlick = shouldFlick;
        this.onFlick = onFlick;
    }

    @Override
    public void start() {
        flickTriggered = false;
        timer.reset();
        // Evaluate condition at start
        shouldFlickResult = shouldFlick.getAsBoolean();
    }

    @Override
    public void update() {
        if (!shouldFlickResult) return; // Nothing to do

        // Wait for flipper to be ready, then trigger
        if (!flickTriggered && spindexer.isReadyToFlip()) {
            spindexer.triggerFlick();
            flickTriggered = true;
            if (onFlick != null) {
                onFlick.run();
            }
            timer.reset();
        }
    }

    @Override
    public boolean isComplete() {
        if (!shouldFlickResult) {
            return true; // Skip - no flick needed
        }
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
        if (!shouldFlickResult) return "MaybeFlick[skip]";
        return flickTriggered ? "MaybeFlick" : "MaybeFlick[wait]";
    }
}
