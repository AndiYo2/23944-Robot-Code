package framework.actions;

import framework.Action;
import utility.Shooting.ShootingSequenceManager;

/**
 * Action that executes the shooting sequence using the ShootingSequenceManager.
 * Completes when the shooting sequence finishes executing.
 */
public class ShootAction implements Action {
    private final ShootingSequenceManager sequenceManager;

    public ShootAction(ShootingSequenceManager sequenceManager) {
        this.sequenceManager = sequenceManager;
    }

    @Override
    public void start() {
        sequenceManager.startShootingSequence();
    }

    @Override
    public void update() {
        // ShootingSequenceManager.update() is called in the main loop
    }

    @Override
    public boolean isComplete() {
        return !sequenceManager.isExecuting();
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
