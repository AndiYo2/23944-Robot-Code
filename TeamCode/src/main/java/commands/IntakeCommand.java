package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;

/**
 * Runs the intake for a specified duration.
 * Automatically stops the intake when the duration has elapsed.
 */
public class IntakeCommand extends CommandBase {
    private final Intake intake;
    private final double duration;
    private final ElapsedTime timer = new ElapsedTime();

    public IntakeCommand(Intake intake, double duration) {
        this.intake = intake;
        this.duration = duration;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.runIntake();
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= duration;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
