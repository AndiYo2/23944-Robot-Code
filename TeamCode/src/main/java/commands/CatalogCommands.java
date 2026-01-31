package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import Constants.EnumConstants;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.SpindexerAndMotifStatus;

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
     * without sorting by color. Tracks balls as Purple for sorted mode compatibility.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @return a command that catalogs balls in fast mode
     */
    public static Command catalogFast(Spindexer spindexer, Intake intake) {
        return new SequentialCommandGroup(
            // Rotate ball 1 from slot 0 to shooter slot (slot 1)
            new RotateCCWCommand(spindexer),
            // Flick ball 1 out, intake ball 2 into slot 0
            new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .35)),
            // Rotate ball 2 to shooter slot (slot 1)
            new RotateCCWCommand(spindexer),
            // Intake ball 3 into slot 0 (reverse intake to spit extras)
            new IntakeCommand(intake, .5, true),
            // Set final pattern: all 3 slots tracked as Purple
            new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple))
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

        // Find which intake position has the first ball to shoot (motifPattern[0])
        int firstBallPos = 0;
        for (int i = 0; i < 3; i++) {
            if (intakeColors[i] == motifPattern[0]) {
                firstBallPos = i;
                break;
            }
        }

        // The final pattern to set after all physical movements complete.
        // In all cases: motifPattern[0] is flicked, motifPattern[1] ends in slot 1,
        // motifPattern[2] ends in slot 0 or slot 2.
        final EnumConstants.BallColor[] finalPattern;

        // Rotation 1: move first intake ball from slot 0 to slot 1
        sequence.addCommands(new RotateCCWCommand(spindexer));

        if (firstBallPos == 0) {
            // BEST CASE: intake[0] == motifPattern[0], flick immediately
            sequence.addCommands(new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .35)));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new IntakeCommand(intake, .5, true));

            if (intakeColors[1] == motifPattern[1]) {
                // motifPattern[1] already in slot 1, done
                finalPattern = new EnumConstants.BallColor[]{motifPattern[2], motifPattern[1], EnumConstants.BallColor.None};
            } else {
                // Extra CCW to bring motifPattern[1] (intake[2]) into slot 1
                sequence.addCommands(new RotateCCWCommand(spindexer));
                finalPattern = new EnumConstants.BallColor[]{EnumConstants.BallColor.None, motifPattern[1], motifPattern[2]};
            }

        } else if (firstBallPos == 1) {
            // intake[1] == motifPattern[0], need to wait for 2nd ball before flicking
            sequence.addCommands(new IntakeCommand(intake, .35));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new FlickCommand(spindexer).alongWith(new IntakeCommand(intake, .5, true)));

            if (intakeColors[0] == motifPattern[1]) {
                // CW to bring intake[0] from slot 2 to slot 1
                sequence.addCommands(new RotateCWCommand(spindexer));
                finalPattern = new EnumConstants.BallColor[]{EnumConstants.BallColor.None, motifPattern[1], motifPattern[2]};
            } else {
                // CCW to bring intake[2] from slot 0 to slot 1
                sequence.addCommands(new RotateCCWCommand(spindexer));
                finalPattern = new EnumConstants.BallColor[]{motifPattern[2], motifPattern[1], EnumConstants.BallColor.None};
            }

        } else {
            // WORST CASE: intake[2] == motifPattern[0], must load all 3 first
            sequence.addCommands(new IntakeCommand(intake, .35));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new IntakeCommand(intake, .5, true));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new FlickCommand(spindexer));

            if (intakeColors[0] == motifPattern[1]) {
                // CCW to bring intake[0] from slot 0 to slot 1
                sequence.addCommands(new RotateCCWCommand(spindexer));
                finalPattern = new EnumConstants.BallColor[]{motifPattern[2], motifPattern[1], EnumConstants.BallColor.None};
            } else {
                // CW to bring intake[1] from slot 2 to slot 1
                sequence.addCommands(new RotateCWCommand(spindexer));
                finalPattern = new EnumConstants.BallColor[]{EnumConstants.BallColor.None, motifPattern[1], motifPattern[2]};
            }
        }

        // Set the authoritative final pattern in one shot, overwriting any
        // intermediate state from rotation/flick commands
        sequence.addCommands(new InstantCommand(() ->
            SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                finalPattern[0], finalPattern[1], finalPattern[2])));

        return sequence;
    }

}