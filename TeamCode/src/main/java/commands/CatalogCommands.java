package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import Constants.EnumConstants;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.DualBallDetector;
import utility.SpindexerAndMotifStatus;

import java.util.function.Consumer;

/**
 * Factory class for composing ball cataloging command sequences.
 * Provides static methods to create cataloging patterns that organize
 * balls from intake into the spindexer for shooting.
 */
public class CatalogCommands {

    /**
     * Fast catalog - doesn't care about color order.
     * Sequence: rotate → parallel(flick, intake) → rotate → intake
     *
     * This is the simpler cataloging mode that just moves balls into position
     * without sorting by color.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @return a command that catalogs balls in fast mode
     */
    public static Command catalogFast(Spindexer spindexer, Intake intake) {
        return new SequentialCommandGroup(
            new RotateCCWCommand(spindexer),
            new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .35)),
            new RotateCCWCommand(spindexer),
            new IntakeCommand(intake, .5)
        );
    }

    /**
     * Sorted catalog - handles all 9 motif×intake combinations plus edge cases.
     * Includes ball tracking updates via InstantCommands.
     *
     * After this command completes:
     * - motifPattern[0] has been flicked into the shooter
     * - motifPattern[1] is in slot 1 (ready to flick next)
     * - motifPattern[2] is in slot 0 or 2 (can rotate to slot 1)
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @param motifPattern the desired shooting order [first, second, third]
     * @param intakeColors the order balls come in from intake [first, second, third]
     * @return a command that catalogs balls in sorted mode
     */
    public static Command catalogSorted(Spindexer spindexer, Intake intake, EnumConstants.BallColor[] motifPattern,
                                         EnumConstants.BallColor[] intakeColors) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        // Find which intake position has the first ball to shoot
        int firstBallPos = 0;
        for (int i = 0; i < 3; i++) {
            if (intakeColors[i] == motifPattern[0]) {
                firstBallPos = i;
                break;
            }
        }

        // Track ball 0 entering slot 0
        sequence.addCommands(new InstantCommand(() ->
            SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[0])));

        // Rotation 1: ball 0 → slot 1
        sequence.addCommands(new RotateCCWCommand(spindexer));

        if (firstBallPos == 0) {
            // BEST CASE: First intake ball is first to shoot - flick early (parallel with intake)
            // 2 rotations if intakeColors[1] == motifPattern[1], else 3
            sequence.addCommands(new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .35)));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[1])));

            // Rotation 2: ball 1 → slot 1
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new IntakeCommand(intake, .5));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[2])));

            // After: slot 0 = intake[2], slot 1 = intake[1], slot 2 = empty
            if (intakeColors[1] != motifPattern[1]) {
                // intake[2] is motifPattern[1], need to rotate CCW
                // Rotation 3
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
            // Done: 2 or 3 rotations

        } else if (firstBallPos == 1) {
            // Second intake ball is first to shoot - flick after 2nd rotate
            // 3 rotations total
            sequence.addCommands(new IntakeCommand(intake, .35));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[1])));

            // Rotation 2: ball 0 → slot 2, ball 1 → slot 1
            sequence.addCommands(new RotateCCWCommand(spindexer));

            // Flick ball 1 (parallel with intake)
            sequence.addCommands(new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .5)));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[2])));

            // After: slot 0 = intake[2], slot 1 = empty, slot 2 = intake[0]
            // Need motifPattern[1] in slot 1
            if (intakeColors[0] == motifPattern[1]) {
                // Rotation 3: CW to bring intake[0] from slot 2 to slot 1
                sequence.addCommands(new RotateCWCommand(spindexer));
            } else {
                // Rotation 3: CCW to bring intake[2] from slot 0 to slot 1
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
            // Done: 3 rotations

        } else {
            // WORST CASE: Third intake ball is first to shoot - must load all first
            // 4 rotations total
            sequence.addCommands(new IntakeCommand(intake, .35));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[1])));

            // Rotation 2: ball 0 → slot 2, ball 1 → slot 1
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new IntakeCommand(intake, .5));
            sequence.addCommands(new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(0, intakeColors[2])));

            // After: slot 0 = intake[2], slot 1 = intake[1], slot 2 = intake[0]
            // Rotation 3: CCW to bring intake[2] to slot 1
            sequence.addCommands(new RotateCCWCommand(spindexer));

            // After: slot 0 = intake[0], slot 1 = intake[2], slot 2 = intake[1]
            sequence.addCommands(new FlickCommand(spindexer));

            // After flick: slot 0 = intake[0], slot 1 = empty, slot 2 = intake[1]
            // Rotation 4: get motifPattern[1] to slot 1
            if (intakeColors[0] == motifPattern[1]) {
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                sequence.addCommands(new RotateCWCommand(spindexer));
            }
            // Done: 4 rotations
        }

        return sequence;
    }

    /**
     * Simplified fast catalog with default callback.
     * Useful when you don't need to track scan results externally.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @return a command that catalogs balls in fast mode
     */
    public static Command catalogFastSimple(Spindexer spindexer, Intake intake) {
        return catalogFast(spindexer, intake);
    }
}