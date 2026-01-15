package framework.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import framework.Action;
import utility.managers.SpindexerManager;

/**
 * Action that initiates ball cataloging using the SpindexerManager.
 * Completes when the cataloging process returns to Idle state OR timeout is reached.
 */
public class CatalogAction implements Action {
    private final SpindexerManager spindexerManager;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    /** Default timeout for cataloging (seconds) */
    public static final double DEFAULT_TIMEOUT = 10.0;

    public CatalogAction(SpindexerManager spindexerManager) {
        this(spindexerManager, DEFAULT_TIMEOUT);
    }

    public CatalogAction(SpindexerManager spindexerManager, double timeoutSeconds) {
        this.spindexerManager = spindexerManager;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.timedOut = false;
    }

    @Override
    public void start() {
        timer.reset();
        timedOut = false;
        spindexerManager.triggerCataloging();
    }

    @Override
    public void update() {
        // Check for timeout
        if (timer.seconds() >= timeoutSeconds && !spindexerManager.isIdle()) {
            timedOut = true;
        }
    }

    @Override
    public boolean isComplete() {
        return spindexerManager.isIdle() || timedOut;
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
        return timedOut ? "Catalog(TIMEOUT)" : "Catalog";
    }
}
