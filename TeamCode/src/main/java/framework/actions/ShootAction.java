package framework.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import framework.Action;
import utility.managers.SpindexerManager;

/**
 * Action that executes the shooting sequence using the SpindexerManager.
 * Completes when the shooting sequence finishes executing OR timeout is reached.
 */
public class ShootAction implements Action {
    private final SpindexerManager spindexerManager;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean hasStarted;
    private boolean timedOut;

    /** Default timeout for shooting sequence (seconds) */
    public static final double DEFAULT_TIMEOUT = 15.0;

    public ShootAction(SpindexerManager spindexerManager) {
        this(spindexerManager, DEFAULT_TIMEOUT);
    }

    public ShootAction(SpindexerManager spindexerManager, double timeoutSeconds) {
        this.spindexerManager = spindexerManager;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.hasStarted = false;
        this.timedOut = false;
    }

    @Override
    public void start() {
        timer.reset();
        hasStarted = false;
        timedOut = false;
        spindexerManager.triggerShooting();
    }

    @Override
    public void update() {
        // Keep trying to start the sequence if it hasn't started yet
        if (!hasStarted && !spindexerManager.isExecuting()) {
            spindexerManager.triggerShooting();
        }

        // Track when it actually starts
        if (!hasStarted && spindexerManager.isExecuting()) {
            hasStarted = true;
        }

        // Check for timeout
        if (timer.seconds() >= timeoutSeconds && spindexerManager.isExecuting()) {
            timedOut = true;
        }
    }

    @Override
    public boolean isComplete() {
        // Complete if sequence started and finished, or if timed out
        return (hasStarted && !spindexerManager.isExecuting()) || timedOut;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    /** Returns true if the action completed due to timeout */
    public boolean didTimeout() {
        return timedOut;
    }

    @Override
    public String getName() {
        return timedOut ? "Shoot(TIMEOUT)" : "Shoot";
    }
}
