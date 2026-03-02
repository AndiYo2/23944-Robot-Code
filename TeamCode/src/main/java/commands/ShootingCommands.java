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

    public static Command slowShootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        return shootAllBallsWithTime(shooter, spindexer, ballCount, 0.30);
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
            // Ball 1
            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
            );

            // Ball 2 + pipeline transition to ball 3
            // Spindexer pushes ball 2 up
            sequence.addCommands(new ExtendSpindexerFlipperCommand(spindexer));

            // Two parallel lanes:
            //   Shooter:   extend (fires ball 2) → retract
            //   Spindexer: wait(halfway) → retract → wait(halfway) → rotate CCW
            long shooterHalfwayMs = (long)(ShootingSequenceConstants.SHOOTER_EXTEND_HALFWAY * 1000);

            sequence.addCommands(
                new ExtendShooterFlipperCommand(shooter, flickTime)
                    .andThen(new RetractShooterFlipperCommand(shooter))
                    .alongWith(
                        new WaitCommand(shooterHalfwayMs)
                            .andThen(new InstantCommand(() -> {
                                spindexer.retractFlipper();
                                spindexer.rotateCCW();
                            }, spindexer))
                            .andThen(new WaitCommand((long)(ShootingSequenceConstants.SPINDEXER_ROTATION_TIME * 1000)))
                    )
            );

            // Ball 3
            sequence.addCommands(
                new ExtendSpindexerFlipperCommand(spindexer),
                new ExtendShooterFlipperCommand(shooter, flickTime),
                new RetractShooterFlipperCommand(shooter)
                    .alongWith(new RetractSpindexerFlipperCommand(spindexer))
            );
        }

        sequence.addCommands(new SpindexerResetCommand(spindexer));
        return sequence;
    }

}
