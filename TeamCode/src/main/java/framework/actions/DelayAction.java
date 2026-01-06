package framework.actions;

import com.pedropathing.util.Timer;

import framework.Action;

/**
 * Action that waits for a specified duration before completing.
 * Useful for adding delays between actions in a sequence.
 */
public class DelayAction implements Action {
    private final double delaySeconds;
    private final Timer timer;

    /**
     * Creates a delay action.
     *
     * @param delaySeconds the duration to wait in seconds
     */
    public DelayAction(double delaySeconds) {
        this.delaySeconds = delaySeconds;
        this.timer = new Timer();
    }

    @Override
    public void start() {
        timer.resetTimer();
    }

    @Override
    public void update() {
        // Timer automatically tracks elapsed time
    }

    @Override
    public boolean isComplete() {
        return timer.getElapsedTimeSeconds() >= delaySeconds;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return "Delay[" + delaySeconds + "s]";
    }
}
