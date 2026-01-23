package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import subsystems.Intake;

/**
 * Command that stops the intake.
 * Completes immediately.
 */
public class IntakeStopCommand extends InstantCommand {

    public IntakeStopCommand(Intake intake) {
        super(intake::stopIntake, intake);
    }
}
