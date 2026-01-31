package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.util.ElapsedTime;
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
 * In Sorted mode, polls sensors each loop until all 3 detect a ball
 * (up to 0.5s timeout), then reads colors and builds the sorted sequence.
 * Falls back to fast catalog if sensors don't read in time or motif is unset.
 */
public class CatalogModeCommand extends CommandBase {
    private static final double SENSOR_TIMEOUT = 0.5;

    private final Spindexer spindexer;
    private final Intake intake;
    private Command actualCatalogCommand;
    private boolean waitingForSensors;
    private final ElapsedTime sensorTimer = new ElapsedTime();

    public CatalogModeCommand(Spindexer spindexer, Intake intake) {
        this.spindexer = spindexer;
        this.intake = intake;
        addRequirements(spindexer, intake);
    }

    @Override
    public void initialize() {
        actualCatalogCommand = null;
        waitingForSensors = false;

        if (SpindexerConstants.currentMode == EnumConstants.ShootingMode.Sorted) {
            // Check if motif pattern has been detected
            boolean motifValid =
                    SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0) != EnumConstants.BallColor.None
                 || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1) != EnumConstants.BallColor.None
                 || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2) != EnumConstants.BallColor.None;

            if (motifValid && allSensorsReady()) {
                // Sensors already have valid readings — start sorted catalog immediately
                startSortedCatalog();
            } else if (motifValid) {
                // Motif detected but sensors not ready — wait for them
                waitingForSensors = true;
                sensorTimer.reset();
            } else {
                // No motif detected — fall back to fast
                startFastCatalog();
            }
        } else {
            startFastCatalog();
        }
    }

    @Override
    public void execute() {
        if (waitingForSensors) {
            if (allSensorsReady()) {
                waitingForSensors = false;
                startSortedCatalog();
            } else if (sensorTimer.seconds() >= SENSOR_TIMEOUT) {
                // Timeout — fall back to fast catalog
                waitingForSensors = false;
                startFastCatalog();
            }
            return;
        }

        if (actualCatalogCommand != null) {
            actualCatalogCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (waitingForSensors) return false;
        return actualCatalogCommand != null && actualCatalogCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (actualCatalogCommand != null) {
            actualCatalogCommand.end(interrupted);
        }
    }

    private boolean allSensorsReady() {
        RobotHardware robot = RobotHardware.getInstance();
        return robot.intakeSensorPair.detectBall().color != EnumConstants.BallColor.None
            && robot.transferSensorPair.detectBall().color != EnumConstants.BallColor.None
            && robot.rampSensorPair.detectBall().color != EnumConstants.BallColor.None;
    }

    private void startSortedCatalog() {
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
        actualCatalogCommand.initialize();
    }

    private void startFastCatalog() {
        actualCatalogCommand = CatalogCommands.catalogFast(spindexer, intake);
        actualCatalogCommand.initialize();
    }
}
