package framework.actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import framework.Action;
import utility.CatalogManager;
import Constants.EnumConstants;

/**
 * Action that initiates ball cataloging using the CatalogManager.
 * Completes when the cataloging process returns to Idle state OR timeout is reached.
 */
public class CatalogAction implements Action {
    private final CatalogManager catalogManager;
    private final double timeoutSeconds;
    private ElapsedTime timer;
    private boolean timedOut;

    /** Default timeout for cataloging (seconds) */
    public static final double DEFAULT_TIMEOUT = 10.0;

    public CatalogAction(CatalogManager catalogManager) {
        this(catalogManager, DEFAULT_TIMEOUT);
    }

    public CatalogAction(CatalogManager catalogManager, double timeoutSeconds) {
        this.catalogManager = catalogManager;
        this.timeoutSeconds = timeoutSeconds;
        this.timer = new ElapsedTime();
        this.timedOut = false;
    }

    @Override
    public void start() {
        timer.reset();
        timedOut = false;
        catalogManager.initiateCataloging();
    }

    @Override
    public void update() {
        // Check for timeout
        if (timer.seconds() >= timeoutSeconds &&
            catalogManager.getState() != EnumConstants.CatalogingCases.IDLE) {
            timedOut = true;
        }
    }

    @Override
    public boolean isComplete() {
        return catalogManager.getState() == EnumConstants.CatalogingCases.IDLE || timedOut;
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
