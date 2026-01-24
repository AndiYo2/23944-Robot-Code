package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import Constants.SpindexerConstants;

/**
 * Runs the intake for a specified duration, then stops.
 */
public class IntakeAction implements SpindexerAction {
    private final Intake intake;
    private final double duration;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeAction(Intake intake) {
        this(intake, SpindexerConstants.INTAKE_TIMING);
    }

    public IntakeAction(Intake intake, double duration) {
        this.intake = intake;
        this.duration = duration;
    }

    @Override
    public void start() {
        intake.runIntake();
        timer.reset();
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return timer.seconds() >= duration;
    }

    @Override
    public void end() {
        intake.stopIntake();
    }

    @Override
    public String getName() {
        return String.format("Intake[%.2fs]", duration);
    }
}
