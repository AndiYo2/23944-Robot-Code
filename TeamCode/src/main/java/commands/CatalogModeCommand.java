package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Command;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotHardware;

/**
 * TeleOp catalog command — always fast mode with sensor-gated intake.
 * No background sensor polling; the spindexer distance sensor is read
 * inside IntakeCommand.isFinished() to finish early on ball detection.
 */
public class CatalogModeCommand extends CommandBase {
    private final Spindexer spindexer;
    private final Intake intake;
    private Command actualCatalogCommand;

    public CatalogModeCommand(Spindexer spindexer, Intake intake) {
        this.spindexer = spindexer;
        this.intake = intake;
        addRequirements(spindexer, intake);
    }

    @Override
    public void initialize() {
        RobotHardware robot = RobotHardware.getInstance();
        actualCatalogCommand = CatalogCommands.catalogFast(spindexer, intake, robot.spindexerSensorPair);
        actualCatalogCommand.initialize();
    }

    @Override
    public void execute() {
        if (actualCatalogCommand != null) {
            actualCatalogCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return actualCatalogCommand != null && actualCatalogCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (actualCatalogCommand != null) {
            actualCatalogCommand.end(interrupted);
        }
    }
}
