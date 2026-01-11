package framework.actions;

import framework.Action;
import utility.CatalogManager;
import Constants.EnumConstants;

/**
 * Action that initiates ball cataloging using the CatalogManager.
 * Completes when the cataloging process returns to Idle state.
 */
public class CatalogAction implements Action {
    private final CatalogManager catalogManager;

    public CatalogAction(CatalogManager catalogManager) {
        this.catalogManager = catalogManager;
    }

    @Override
    public void start() {
        catalogManager.initiateCataloging();
    }

    @Override
    public void update() {
        // CatalogManager.update() is called in the main loop
    }

    @Override
    public boolean isComplete() {
        return catalogManager.getState() == EnumConstants.CatalogingCases.Idle;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return "Catalog";
    }
}
