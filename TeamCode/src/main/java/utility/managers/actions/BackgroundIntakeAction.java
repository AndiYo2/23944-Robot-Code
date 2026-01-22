package utility.managers.actions;

import subsystems.Intake;
import Constants.SpindexerConstants;

/**
 * Starts intake and immediately completes (non-blocking).
 * Intake runs in background and auto-stops after duration via Intake.periodic().
 * Use for the last intake in a sequence so cataloging finishes immediately.
 */
public class BackgroundIntakeAction implements SpindexerAction {
    private final Intake intake;
    private final double duration;

    public BackgroundIntakeAction(Intake intake) {
        this(intake, SpindexerConstants.INTAKE_TIMING);
    }

    public BackgroundIntakeAction(Intake intake, double duration) {
        this.intake = intake;
        this.duration = duration;
    }

    @Override
    public void start() {
        intake.runIntakeBeltForDuration(duration);
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return true;  // Immediately complete - intake runs in background
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return String.format("BgIntake[%.2fs]", duration);
    }
}
