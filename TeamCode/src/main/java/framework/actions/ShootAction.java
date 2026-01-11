package framework.actions;

import framework.Action;
import utility.ShootingSequenceManager;

/**
 * Action that executes the shooting sequence using the ShootingSequenceManager.
 * Completes when the shooting sequence finishes executing.
 */
public class ShootAction implements Action {
    private final ShootingSequenceManager sequenceManager;
    private boolean hasStarted;

    public ShootAction(ShootingSequenceManager sequenceManager) {
        this.sequenceManager = sequenceManager;
        this.hasStarted = false;
    }

    @Override
    public void start() {
        hasStarted = false;
        sequenceManager.startShootingSequence();
    }

    @Override
    public void update() {
        // Keep trying to start the sequence if it hasn't started yet
        if (!hasStarted && !sequenceManager.isExecuting()) {
            sequenceManager.startShootingSequence();
        }

        // Track when it actually starts
        if (!hasStarted && sequenceManager.isExecuting()) {
            hasStarted = true;
        }
    }

    @Override
    public boolean isComplete() {
        // Only complete if the sequence has started AND finished
        return hasStarted && !sequenceManager.isExecuting();
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return "Shoot";
    }
}
