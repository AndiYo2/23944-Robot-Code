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
 * Guaranteed sorted auto catalog command. Combines sensor-waiting from
 * CatalogModeCommand with auto timing variants from AutoCatalogModeCommand.
 *
 * Designed to run inside .parallel() groups alongside path following,
 * so sensor-waiting time overlaps with driving time.
 *
 * Key improvements over using CatalogModeCommand in auto:
 * - Forces fresh bulk-safe sensor reads (not stale round-robin cache)
 * - Uses auto catalog variants (faster timings)
 * - Validates all 3 sensor colors are non-None before sorted catalog
 * - Falls back to fast auto catalog if validation fails
 * - Records debug telemetry
 */
public class GuaranteeSortedAutoCatalogCommand extends CommandBase {
    private static final double SENSOR_TIMEOUT = 0.5;
    public static String lastCatalogDebug = "";

    private final Spindexer spindexer;
    private final Intake intake;
    private Command actualCatalogCommand;
    private boolean waitingForSensors;
    private final ElapsedTime sensorTimer = new ElapsedTime();

    public GuaranteeSortedAutoCatalogCommand(Spindexer spindexer, Intake intake) {
        this.spindexer = spindexer;
        this.intake = intake;
        addRequirements(spindexer, intake);
    }

    @Override
    public void initialize() {
        actualCatalogCommand = null;
        waitingForSensors = false;

        boolean motifValid =
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0) != EnumConstants.BallColor.None
             || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1) != EnumConstants.BallColor.None
             || SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2) != EnumConstants.BallColor.None;

        if (!motifValid) {
            startFastCatalog();
            lastCatalogDebug = "Fast (no motif)";
            return;
        }

        pollAllSensors();

        if (allSensorsReady()) {
            startSortedCatalog();
        } else {
            waitingForSensors = true;
            sensorTimer.reset();
        }
    }

    @Override
    public void execute() {
        if (waitingForSensors) {
            pollAllSensors();

            if (allSensorsReady()) {
                waitingForSensors = false;
                startSortedCatalog();
            } else if (sensorTimer.seconds() >= SENSOR_TIMEOUT) {
                waitingForSensors = false;
                startFastCatalog();
                lastCatalogDebug = "Fast (sensor timeout)";
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

    private void pollAllSensors() {
        RobotHardware robot = RobotHardware.getInstance();
        robot.intakeSensorPair.updateCacheBulkSafe();
        robot.transferSensorPair.updateCacheBulkSafe();
        robot.rampSensorPair.updateCacheBulkSafe();
    }

    private boolean allSensorsReady() {
        RobotHardware robot = RobotHardware.getInstance();
        return robot.intakeSensorPair.quickCheck().color != EnumConstants.BallColor.None
            && robot.transferSensorPair.quickCheck().color != EnumConstants.BallColor.None
            && robot.rampSensorPair.quickCheck().color != EnumConstants.BallColor.None;
    }

    private void startSortedCatalog() {
        pollAllSensors();

        RobotHardware robot = RobotHardware.getInstance();
        EnumConstants.BallColor[] motifPattern = {
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(0),
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(1),
                SpindexerAndMotifStatus.MotifPattern.getBallColorInSlotX(2)
        };
        EnumConstants.BallColor[] intakeColors = {
                robot.intakeSensorPair.quickCheck().color,
                robot.transferSensorPair.quickCheck().color,
                robot.rampSensorPair.quickCheck().color
        };

        boolean allValid = intakeColors[0] != EnumConstants.BallColor.None
                        && intakeColors[1] != EnumConstants.BallColor.None
                        && intakeColors[2] != EnumConstants.BallColor.None;

        lastCatalogDebug = "I:[" + intakeColors[0] + "," + intakeColors[1] + "," + intakeColors[2]
                + "] M:[" + motifPattern[0] + "," + motifPattern[1] + "," + motifPattern[2] + "]";

        if (allValid) {
            actualCatalogCommand = CatalogCommands.catalogSortedAuto(spindexer, intake, motifPattern, intakeColors);
            lastCatalogDebug += " -> Sorted";
        } else {
            actualCatalogCommand = CatalogCommands.catalogFastAuto(spindexer, intake);
            lastCatalogDebug += " -> Fast (None detected)";
        }
        actualCatalogCommand.initialize();
    }

    private void startFastCatalog() {
        actualCatalogCommand = CatalogCommands.catalogFastAuto(spindexer, intake);
        actualCatalogCommand.initialize();
    }
}
