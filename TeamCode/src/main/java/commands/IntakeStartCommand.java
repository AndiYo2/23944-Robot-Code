package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import subsystems.Intake;

public class IntakeStartCommand extends InstantCommand {

    public IntakeStartCommand(Intake intake) {
        super(intake::runIntake, intake);
    }
}
