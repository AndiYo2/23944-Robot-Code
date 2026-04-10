package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import Constants.ShootingSequenceConstants;
import subsystems.Shooter;
import subsystems.Spindexer;

/**
 * Factory class for composing shooting command sequences.
 * Uses atomic extend/retract commands with halfway pipelining
 * to overlap spindexer retract/rotate with shooter operations.
 */
public class ShootingCommands {

    /**
     * Shoots 3 balls.
     * Cataloging always ends at [m2, m1, None], so this uses a fixed sequence.
     */
    public static Command shootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return shootAllBalls(shooter, spindexer, 3);
    }


    public static Command shootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return shootAllBallsWithTime(shooter, spindexer, ballCount, ShootingSequenceConstants.SHOOTER_FLICK_TIME);
    }

    public static Command slowShootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return slowShootAllBalls(shooter, spindexer, 3);
    }

    public static Command superSlowShootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return superSlowShootAllBalls(shooter, spindexer, 3);
    }

    public static Command slowShootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return shootAllBallsWithTime(shooter, spindexer, ballCount, 0.375);
    }

    public static Command superSlowShootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return shootAllBallsWithTime(shooter, spindexer, ballCount, 0.5);
    }

    private static Command shootAllBallsWithTime(Shooter shooter, Spindexer spindexer, int ballCount, double flickTime) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        if (ballCount <= 0) {
            return sequence;
        }

        if (ballCount == 1) {
            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
            );
        } else if (ballCount == 2) {
            sequence.addCommands(
                // Ball 1
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter),
                // Ball 2
                new ExtendSpindexerFlipperCommand(spindexer),
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
                    .alongWith(new RetractSpindexerFlipperCommand(spindexer))
            );
        } else if (ballCount == 3) {
            // Ball 1: fire from slot 1
            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
            );

            // Ball 2: spindexer pushes ball up
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer));

            // Two parallel lanes:
            //   P1: extend shooter (fires ball 2) → retract shooter
            //   P2: wait(halfway) → retract spindexer flipper → rotate spindexer
            long shooterHalfwayMs = (long)(ShootingSequenceConstants.SHOOTER_EXTEND_HALFWAY * 1000);

            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime)
                    .andThen(new RetractShooterFlipperCommand(shooter))
                    .alongWith(
                        new WaitCommand(shooterHalfwayMs)
                            .andThen(new RetractSpindexerFlipperCommand(spindexer))
                            .andThen(new RotateCCWCommand(spindexer))
                    )
            );

            // Ball 3: push up, fire, retract spindexer
            sequence.addCommands(
                new ExtendSpindexerFlipperCommand(spindexer),
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter),
                new RetractSpindexerFlipperCommand(spindexer)
            );
        }

        sequence.addCommands(new SpindexerResetCommand(spindexer));
        return sequence;
    }

}
