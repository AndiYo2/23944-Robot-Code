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
 * All variants use the same physical shoot mechanism (SHOOTER_FLICK_TIME).
 * Slow variants add delays between shots rather than changing flick timing.
 */
public class ShootingCommands {

    public static Command shootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return shootAllBalls(shooter, spindexer, 3);
    }

    public static Command shootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return buildShootSequence(shooter, spindexer, ballCount, 0);
    }

    public static Command slowShootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return slowShootAllBalls(shooter, spindexer, 3);
    }

    public static Command superSlowShootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return superSlowShootAllBalls(shooter, spindexer, 3);
    }

    public static Command slowShootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return buildShootSequence(shooter, spindexer, ballCount, ShootingSequenceConstants.SLOW_SHOOT_DELAY);
    }

    public static Command superSlowShootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return buildShootSequence(shooter, spindexer, ballCount, ShootingSequenceConstants.SUPER_SLOW_SHOOT_DELAY);
    }

    private static Command buildShootSequence(Shooter shooter, Spindexer spindexer, int ballCount, double delayBetweenShots) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        double flickTime = ShootingSequenceConstants.SHOOTER_FLICK_TIME;
        long delayMs = (long)(delayBetweenShots * 1000);

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
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
            );
            if (delayMs > 0) {
                sequence.addCommands(new WaitCommand(delayMs));
            }
            sequence.addCommands(
                new ExtendSpindexerFlipperCommand(spindexer),
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
                    .alongWith(new RetractSpindexerFlipperCommand(spindexer))
            );
        } else if (ballCount == 3) {
            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
            );

            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer));


            long equalizationMs = (long)(ShootingSequenceConstants.SHOT_EQUALIZATION_DELAY * 1000);
            long laneAPreFireWait = Math.max(delayMs, equalizationMs);
            long shooterHalfwayMs = (long)(ShootingSequenceConstants.SHOOTER_EXTEND_HALFWAY * 1000);

            sequence.addCommands(
                new WaitCommand(laneAPreFireWait)
                    .andThen(new ExtendShooterFlipperCommand(shooter, flickTime))
                    .andThen(new RetractShooterFlipperCommand(shooter))
                    .alongWith(
                        new WaitCommand(shooterHalfwayMs)
                            .andThen(new RetractSpindexerFlipperCommand(spindexer))
                            .andThen(new RotateCCWCommand(spindexer))
                    )
            );

            if (delayMs > 0) {
                sequence.addCommands(new WaitCommand(delayMs));
            }

            sequence.addCommands(
                new ExtendSpindexerFlipperCommand(spindexer),
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter),
                new RetractSpindexerFlipperCommand(spindexer)
            );
        }

        sequence.addCommands(
            new SpindexerResetCommand(spindexer),
            new InstantCommand(shooter::resetHoodCompensation)
        );
        return sequence;
    }

}