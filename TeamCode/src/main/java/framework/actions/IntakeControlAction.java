package framework.actions;

import framework.Action;
import subsystems.Intake;

/**
 * Action that controls the intake mechanism.
 * This is an immediate action that completes in one cycle.
 */
public class IntakeControlAction implements Action {
    private final Intake intake;
    private final boolean start; // true = start intake, false = stop intake

    /**
     * Creates an intake control action.
     *
     * @param intake the intake subsystem to control
     * @param start true to start the intake, false to stop it
     */
    public IntakeControlAction(Intake intake, boolean start) {
        this.intake = intake;
        this.start = start;
    }

    @Override
    public void start() {
        if (start) {
            intake.runIntake();
        } else {
            intake.stopIntake();
        }
    }

    @Override
    public void update() {
        // Immediate action, nothing to update
    }

    @Override
    public boolean isComplete() {
        // Completes immediately after start()
        return true;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return start ? "IntakeStart" : "IntakeStop";
    }
}
