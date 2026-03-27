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
 * Auto catalog command - reads sensors immediately and sorts.
 * When in Sorted mode with a valid motif, reads all sensor colors
 * and runs the sorted catalog right away. Falls back to fast only
 * if the motif pattern is unset or mode is Fast.
 */
public class AutoCatalogModeCommand extends CommandBase {
    private final Spindexer spindexer;
    private final Intake intake;
    private Command actualCatalogCommand;

    public AutoCatalogModeCommand(Spindexer spindexer, Intake intake) {
        this.spindexer = spindexer;
        this.intake = intake;
        addRequirements(spindexer, intake);
    }

    @Override
    public void initialize() {
        actualCatalogCommand = null;

        if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
            boolean motifValid =
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0) != EnumConstants.BallColor.None
                 || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1) != EnumConstants.BallColor.None
                 || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2) != EnumConstants.BallColor.None;

            if (motifValid) {
                startSortedCatalog();
            } else {
                startFastCatalog();
            }
        } else {
            startFastCatalog();
        }
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

    private void startSortedCatalog() {
        RobotHardware robot = RobotHardware.getInstance();
        EnumConstants.BallColor[] motifPattern = {
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0),
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)
        };
        EnumConstants.BallColor[] intakeColors = {
                robot.spindexerSensorPair.quickCheck().color,
                robot.transferSensorPair.quickCheck().color,
                robot.rampSensorPair.quickCheck().color
        };
        actualCatalogCommand = CatalogCommands.catalogSortedAuto(spindexer, intake, motifPattern, intakeColors);
        actualCatalogCommand.initialize();
    }

    private void startFastCatalog() {
        actualCatalogCommand = CatalogCommands.catalogFastAuto(spindexer, intake);
        actualCatalogCommand.initialize();
    }
}
