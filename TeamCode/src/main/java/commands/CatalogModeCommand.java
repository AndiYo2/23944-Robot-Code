package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Command;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotHardware;
import utility.SpindexerAndMotifStatus;

/**
 * Catalog command that checks the current shooting mode and executes
 * the appropriate cataloging strategy (Fast or Sorted).
 *
 * This command checks the mode when it initializes, allowing it to
 * respect mode changes made earlier in a command sequence.
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
        // Check mode at execution time and create the appropriate catalog command
        if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
            // Sorted mode - read motif pattern and intake colors
            RobotHardware robot = RobotHardware.getInstance();
            EnumConstants.BallColor[] motifPattern = {
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0),
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)
            };
            EnumConstants.BallColor[] intakeColors = {
                    robot.intakeSensorPair.detectBall().color,
                    robot.transferSensorPair.detectBall().color,
                    robot.rampSensorPair.detectBall().color
            };
            actualCatalogCommand = CatalogCommands.catalogSorted(spindexer, intake, motifPattern, intakeColors);
        } else {
            // Fast mode - simple cataloging
            actualCatalogCommand = CatalogCommands.catalogFast(spindexer, intake);
        }

        // Initialize the selected catalog command
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