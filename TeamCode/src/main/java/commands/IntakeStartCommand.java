package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import subsystems.Intake;

/**
 * Command that starts the intake.
 * Completes immediately - intake continues running until stopped.
 */
public class IntakeStartCommand extends InstantCommand {

    public IntakeStartCommand(Intake intake) {
        super(intake::runIntake, intake);
    }
}
