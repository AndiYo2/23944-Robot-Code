package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
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
            new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new IntakeCommand(intake, .6)),
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
     * Sorted catalog - uses bidirectional rotation (CW and CCW) to always
     * produce the same end state: [motifPattern[2], motifPattern[1], None].
     *
     * Starting from 180° (EMPTY_RESET_DEGREES), uses both CW and CCW during
     * loading to control which slot each ball enters. This guarantees:
     * - motifPattern[0] is flicked into the shooter
     * - motifPattern[1] is in slot 1 (ready to flick next)
     * - motifPattern[2] is in slot 0 (one CCW from slot 1)
     *
     * Because the end state is always the same, shooting is always:
     * fire → flick → CCW → fire → flick → fire → reset
     *
     * 6 cases based on firstBallPos (0/1/2) and remaining ball order (a/b):
     *   1a: 2 rotations  | 1b: 3 rotations  | 2a: 3 rotations
     *   2b: 4 rotations  | 3a: 5 rotations  | 3b: 4 rotations
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

        if (firstBallPos == 0) {
            // intake[0] == motifPattern[0]: flick first ball immediately
            // CCW moves ball 1 to slot 1, flick it to shooter
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new IntakeCommand(intake, .35)));

            if (intakeColors[1] == motifPattern[1]) {
                // Case 1a: intake order matches motif order
                // Ball 2 (m1) → slot 1 via CCW, ball 3 (m2) → slot 0
                // Servo: 180→120→60
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
            } else {
                // Case 1b: remaining balls are swapped
                // CW parks ball 2 (m2) in slot 2, intake ball 3 (m1), CCW puts m1→slot 1 and m2→slot 0
                // Servo: 180→120→180→120
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else if (firstBallPos == 1) {
            // intake[1] == motifPattern[0]: must wait for ball 2 before flicking
            // CW parks ball 1 in slot 2, intake ball 2 (m0), CCW brings m0 to slot 1
            sequence.addCommands(new RotateCWCommand(spindexer));
            sequence.addCommands(new IntakeCommand(intake, .35));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)));

            if (intakeColors[0] == motifPattern[1]) {
                // Case 2a: ball 1 is m1 (second to shoot)
                // CCW moves m1 to slot 1, intake ball 3 (m2) at slot 0
                // Servo: 180→240→180→120
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
            } else {
                // Case 2b: ball 1 is m2 (third to shoot)
                // CW parks m2 in slot 2, intake ball 3 (m1), CCW puts m1→slot 1 and m2→slot 0
                // Servo: 180→240→180→240→180
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else {
            // intake[2] == motifPattern[0]: must load all 3 before flicking

            if (intakeColors[0] == motifPattern[1]) {
                // Case 3a: intake = [m1, m2, m0]
                // CCW loading, then CW×2 to bring m0 to slot 1, flick, CCW to arrange
                // Servo: 180→120→60→120→180→120
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .35));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                // Case 3b: intake = [m2, m1, m0]
                // CW loading, then CCW to bring m0 to slot 1, flick, CCW to arrange
                // Servo: 180→240→300→240→180
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .35));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new IntakeCommand(intake, .5, true));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
        }

        // Final pattern is always the same: m2 in slot 0, m1 in slot 1, slot 2 empty
        sequence.addCommands(new InstantCommand(() ->
            SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                motifPattern[2], motifPattern[1], EnumConstants.BallColor.None)));

        return sequence;
    }

    // ==================== Auto Catalog Variants ====================
    // These assume intake is already running and never stop it mid-sequence.
    // WaitCommands replace IntakeCommands for timing. Only at the very end
    // does the intake switch to ReversedInBeltGo for a fixed duration.

    /**
     * Fast auto catalog - assumes intake is already running.
     * Never stops the intake during the sequence. At the end, switches to
     * ReversedInBeltGo (intake reversed, belt forward) for 0.5s to spit extras.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @return a command that catalogs balls in fast mode without stopping intake
     */
    public static Command catalogFastAuto(Spindexer spindexer, Intake intake) {
        return new SequentialCommandGroup(
            // Rotate ball 1 from slot 0 to shooter slot (slot 1)
            new RotateCCWCommand(spindexer),
            // Flick ball 1 out, wait for ball 2 to arrive (intake already running)
            new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new WaitCommand(600)),
            // Rotate ball 2 to shooter slot (slot 1)
            new RotateCCWCommand(spindexer),
            // Reverse intake to spit extras, belt keeps going to pull ball 3 in
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
     * Sorted auto catalog - assumes intake is already running.
     * Same 6-case sorting logic as catalogSorted, but never stops the intake
     * mid-sequence. WaitCommands replace IntakeCommands for timing. At the very
     * end, switches to ReversedInBeltGo for 0.5s to spit extras.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @param motifPattern the desired shooting order [first, second, third]
     * @param intakeColors the order balls come in from intake [first, second, third]
     * @return a command that catalogs balls in sorted mode without stopping intake
     */
    public static Command catalogSortedAuto(Spindexer spindexer, Intake intake, EnumConstants.BallColor[] motifPattern,
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

        if (firstBallPos == 0) {
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new WaitCommand(350)));

            if (intakeColors[1] == motifPattern[1]) {
                // Case 1a
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                // Case 1b
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(500));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else if (firstBallPos == 1) {
            sequence.addCommands(new RotateCWCommand(spindexer));
            sequence.addCommands(new WaitCommand(350));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)));

            if (intakeColors[0] == motifPattern[1]) {
                // Case 2a
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                // Case 2b
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(500));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else {
            if (intakeColors[0] == motifPattern[1]) {
                // Case 3a
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(350));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(500));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                    .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                // Case 3b
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(350));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand(500));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                    .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
        }

        // Reverse intake to spit extras, belt keeps going to pull last ball in
        sequence.addCommands(new IntakeCommand(intake, .5, true));

        // Final pattern is always the same: m2 in slot 0, m1 in slot 1, slot 2 empty
        sequence.addCommands(new InstantCommand(() ->
            SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                motifPattern[2], motifPattern[1], EnumConstants.BallColor.None)));

        return sequence;
    }

}