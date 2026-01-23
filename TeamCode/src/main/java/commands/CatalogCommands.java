package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import Constants.SpindexerConstants;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.Spindexer;
import utility.DualBallDetector;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Factory class for composing ball cataloging command sequences.
 * Provides static methods to create cataloging patterns that organize
 * balls from intake into the spindexer for shooting.
 */
public class CatalogCommands {

    /**
     * Fast catalog - doesn't care about color order.
     * Sequence: scan → rotate → parallel(flick, intake) → rotate → intake
     *
     * This is the simpler cataloging mode that just moves balls into position
     * without sorting by color.
     *
     * @param spindexer the spindexer subsystem
     * @param shooter the shooter subsystem (for flicking)
     * @param intake the intake subsystem
     * @param sensor1 first ball detector (spindexer slot 0)
     * @param sensor2 second ball detector (transfer)
     * @param sensor3 third ball detector (ramp)
     * @param onScanComplete callback invoked after sensor scan with results
     * @return a command that catalogs balls in fast mode
     */
    public static Command catalogFast(Spindexer spindexer, Shooter shooter,
                                       Intake intake, DualBallDetector sensor1,
                                       DualBallDetector sensor2, DualBallDetector sensor3,
                                       Consumer<ScanSensorsCommand.ScanResults> onScanComplete) {
        return new SequentialCommandGroup(
            // Step 1: Scan sensors to detect ball positions and colors
            new ScanSensorsCommand(sensor1, sensor2, sensor3, onScanComplete),

            // Step 2: Rotate ball 1 from slot 0 to shooter position (slot 1)
            new RotateCCWCommand(spindexer),

            // Step 3: In parallel - flick ball 1 to shooter while intaking ball 2
            new ParallelCommandGroup(
                new FlickCommand(spindexer),
                new IntakeCommand(intake, SpindexerConstants.INTAKE_TIMING)
            ),

            // Step 4: Rotate ball 2 to slot 1
            new RotateCCWCommand(spindexer),

            // Step 5: Intake ball 3 into slot 0
            new IntakeCommand(intake, SpindexerConstants.INTAKE_TIMING)
        );
    }

    /**
     * Sorted catalog - shoots in motif pattern order.
     * More complex with conditional flicks based on ball color matching the motif.
     *
     * This mode ensures balls are positioned to be shot in the correct order
     * based on the alliance's motif pattern.
     *
     * @param spindexer the spindexer subsystem
     * @param shooter the shooter subsystem
     * @param intake the intake subsystem
     * @param sensor1 first ball detector (spindexer slot 0)
     * @param sensor2 second ball detector (transfer)
     * @param sensor3 third ball detector (ramp)
     * @param onScanComplete callback invoked after sensor scan with results
     * @param shouldFlipBall1 condition supplier for whether ball 1 matches motif[0]
     * @param shouldFlipBall2 condition supplier for whether ball 2 should be flipped
     * @param shouldFlipBall3 condition supplier for whether ball 3 should be flipped
     * @param onFlip callback invoked each time a flip occurs (to track state)
     * @return a command that catalogs balls in sorted mode
     */
    public static Command catalogSorted(Spindexer spindexer, Shooter shooter,
                                         Intake intake,
                                         DualBallDetector sensor1,
                                         DualBallDetector sensor2,
                                         DualBallDetector sensor3,
                                         Consumer<ScanSensorsCommand.ScanResults> onScanComplete,
                                         BooleanSupplier shouldFlipBall1,
                                         BooleanSupplier shouldFlipBall2,
                                         BooleanSupplier shouldFlipBall3,
                                         Runnable onFlip) {
        return new SequentialCommandGroup(
            // Step 1: Scan sensors to detect ball positions and colors
            new ScanSensorsCommand(sensor1, sensor2, sensor3, onScanComplete),

            // Step 2: Rotate ball 1 from slot 0 to shooter position (slot 1)
            new RotateCCWCommand(spindexer),

            // Step 3: In parallel - maybe flip ball 1 (if matches motif[0]) + intake ball 2
            new ParallelCommandGroup(
                new PossibleFlickCommand(spindexer, shouldFlipBall1, onFlip),
                new IntakeCommand(intake, SpindexerConstants.INTAKE_TIMING)
            ),

            // Step 4: Rotate ball 2 to slot 1
            new RotateCCWCommand(spindexer),

            // Step 5: In parallel - maybe flip ball 2 + intake ball 3
            new ParallelCommandGroup(
                new PossibleFlickCommand(spindexer, shouldFlipBall2, onFlip),
                new IntakeCommand(intake, SpindexerConstants.INTAKE_TIMING)
            ),

            // Step 6: Rotate ball 3 to slot 1
            new RotateCCWCommand(spindexer),

            // Step 7: Maybe flip ball 3 (no more balls to intake)
            new PossibleFlickCommand(spindexer, shouldFlipBall3, onFlip)
        );
    }

    /**
     * Simplified fast catalog with default callback.
     * Useful when you don't need to track scan results externally.
     *
     * @param spindexer the spindexer subsystem
     * @param shooter the shooter subsystem
     * @param intake the intake subsystem
     * @param sensor1 first ball detector
     * @param sensor2 second ball detector
     * @param sensor3 third ball detector
     * @return a command that catalogs balls in fast mode
     */
    public static Command catalogFastSimple(Spindexer spindexer, Shooter shooter,
                                             Intake intake,
                                             DualBallDetector sensor1,
                                             DualBallDetector sensor2,
                                             DualBallDetector sensor3) {
        return catalogFast(spindexer, shooter, intake, sensor1, sensor2, sensor3,
            results -> { /* No-op callback */ });
    }
}
