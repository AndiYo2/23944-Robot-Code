package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import Constants.EnumConstants;
import subsystems.Intake;
import subsystems.Spindexer;
import Constants.SpindexerConstants;
import utility.DualBallDetector;
import utility.SpindexerAndMotifStatus;

/**
 * Factory class for composing ball cataloging command sequences.
 * Provides static methods to create cataloging patterns that organize
 * balls from intake into the spindexer for shooting.
 */
public class CatalogCommands {


    /**
     * Fast catalog with optional sensor gate — IntakeCommand finishes early
     * when the spindexer distance sensor detects a ball (or timeout).
     */
    public static Command catalogFast(Spindexer spindexer, Intake intake, DualBallDetector sensorGate) {
        return new SequentialCommandGroup(
            new RotateCCWCommand(spindexer),
            new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)),
            new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new IntakeCommand(intake, SpindexerConstants.TELEOP_FIRST_CATALOG_INTAKE_TIME, false, sensorGate)),
            new InstantCommand(() -> intake.runStagingOnly()),
            new RotateCCWCommand(spindexer),
            new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)),
            new IntakeCommand(intake, SpindexerConstants.TELEOP_REVERSE_CATALOG_INTAKE_TIME, true, sensorGate),
            new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple))
        );
    }
    /**
     * Fast auto catalog - stops intake briefly, then manages intake lifecycle.
     * Stops intake for AUTO_CATALOG_STOP_TIME, restarts it (belt stays on throughout),
     * then reverses external intake after last rotate.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @return a command that catalogs balls in fast mode for auto
     */
    public static Command catalogFastAuto(Spindexer spindexer, Intake intake) {
        return new SequentialCommandGroup(
            new InstantCommand(() -> intake.stopIntake()),
            new WaitCommand((long)(SpindexerConstants.AUTO_CATALOG_STOP_TIME * 1000)),
            new InstantCommand(() -> intake.runIntake()),
            new RotateCCWCommand(spindexer),
            new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)),
            new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new WaitCommand((long)(SpindexerConstants.AUTO_FIRST_CATALOG_INTAKE_TIME * 1000))),
            new RotateCCWCommand(spindexer),
            new IntakeCommand(intake, SpindexerConstants.AUTO_REVERSE_CATALOG_INTAKE_TIME, true),
            new InstantCommand(() ->
                SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple,
                    EnumConstants.BallColor.Purple))
        );
    }

    /**
     * Sorted auto catalog - stops intake briefly, then manages intake lifecycle.
     * Stops intake for AUTO_CATALOG_STOP_TIME, restarts it (belt stays on throughout),
     * then reverses external intake after the last rotate. Same 6-case sorting logic
     * as catalogSorted with WaitCommands for timing between rotations.
     *
     * @param spindexer the spindexer subsystem
     * @param intake the intake subsystem
     * @param motifPattern the desired shooting order [first, second, third]
     * @param intakeColors the order balls come in from intake [first, second, third]
     * @return a command that catalogs balls in sorted mode for auto
     */
    public static Command catalogSortedAuto(Spindexer spindexer, Intake intake, EnumConstants.BallColor[] motifPattern,
                                             EnumConstants.BallColor[] intakeColors) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        sequence.addCommands(new InstantCommand(() -> intake.stopIntake()));
        sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_CATALOG_STOP_TIME * 1000)));
        sequence.addCommands(new InstantCommand(() -> intake.runIntake()));

        int firstBallPos = 0;
        for (int i = 0; i < 3; i++) {
            if (intakeColors[i] == motifPattern[0]) {
                firstBallPos = i;
                break;
            }
        }

        if (firstBallPos == 0) {
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)).alongWith(new WaitCommand((long)(SpindexerConstants.AUTO_FIRST_CATALOG_INTAKE_TIME * 1000))));

            if (intakeColors[1] == motifPattern[1]) {
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_SECOND_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else if (firstBallPos == 1) {
            sequence.addCommands(new RotateCWCommand(spindexer));
            sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_FIRST_CATALOG_INTAKE_TIME * 1000)));
            sequence.addCommands(new RotateCCWCommand(spindexer));
            sequence.addCommands(new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)));
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                .andThen(new RetractSpindexerFlipperCommand(spindexer)));

            if (intakeColors[0] == motifPattern[1]) {
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_SECOND_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }

        } else {
            if (intakeColors[0] == motifPattern[1]) {
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_FIRST_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_SECOND_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                    .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            } else {
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_FIRST_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.AUTO_SECOND_CATALOG_INTAKE_TIME * 1000)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
                sequence.addCommands(new WaitCommand((long)(SpindexerConstants.ROTATION_SETTLE_TIME * 1000)));
                sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer)
                    .andThen(new RetractSpindexerFlipperCommand(spindexer)));
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
        }

        sequence.addCommands(new IntakeCommand(intake, SpindexerConstants.AUTO_REVERSE_CATALOG_INTAKE_TIME, true));

        sequence.addCommands(new InstantCommand(() ->
            SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                motifPattern[2], motifPattern[1], EnumConstants.BallColor.None)));

        return sequence;
    }

}