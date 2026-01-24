package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;

/**
 * Factory class for composing shooting command sequences.
 * Provides static methods to create common shooting patterns.
 */
public class ShootingCommands {

    /**
     * Shoots all balls in the spindexer.
     * Sequence for each ball: Fire -> Flick -> RotateToNext (except last ball)
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @param ballCount number of balls to shoot
     * @return a command that shoots all balls
     */
    /**
     * Shoots 3 balls - use this when ball detection hardware is unavailable.
     * Sequence for each ball: Fire -> Flick -> RotateToNext (except last ball)
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @return a command that shoots 3 balls
     */
    public static Command shootThreeBalls(Shooter shooter, Spindexer spindexer) {
        return shootAllBalls(shooter, spindexer, 3);
    }

    public static Command shootAllBalls(Shooter shooter, Spindexer spindexer, int ballCount) {
        SequentialCommandGroup sequence = new SequentialCommandGroup();

        if (ballCount <= 0) {
            return sequence; // Return empty sequence for invalid ball count
        }

        for (int i = 0; i < ballCount - 1; i++) {
            // Fire the current ball
            sequence.addCommands(new FireCommand(shooter));

            // Flick the ball into the shooter
            sequence.addCommands(new FlickCommand(spindexer));

            // If not the last ball, rotate to the next one
            if (i < ballCount - 1) {
                sequence.addCommands(new RotateCCWCommand(spindexer));
            }
        }
        sequence.addCommands(new FireCommand(shooter));

        // Reset spindexer to 300 degrees after shooting
        sequence.addCommands(new SpindexerResetCommand(spindexer));

        return sequence;
    }


    /**
     * Shoots all balls with shooter spinup and turret alignment wait.
     * Waits for the shooter to reach target velocity and turret to be aligned before firing.
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @param turret the turret subsystem
     * @param ballCount number of balls to shoot
     * @param spinupTimeout max seconds to wait for shooter spinup
     * @param alignTimeout max seconds to wait for turret alignment
     * @return a command that waits for spinup and alignment then shoots all balls
     */
    public static Command shootWithSpinup(Shooter shooter, Spindexer spindexer,
                                          Turret turret, int ballCount,
                                          double spinupTimeout, double alignTimeout) {
        return new SequentialCommandGroup(
            new WaitForShooterReadyCommand(shooter, spinupTimeout),
            new WaitForTurretAlignedCommand(turret, alignTimeout),
            shootAllBalls(shooter, spindexer, ballCount)
        );
    }

    /**
     * Shoots all balls after waiting for shooter spinup only.
     * Use this when turret alignment is not needed.
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @param ballCount number of balls to shoot
     * @param spinupTimeout max seconds to wait for shooter spinup
     * @return a command that waits for spinup then shoots all balls
     */
    public static Command shootWithSpinup(Shooter shooter, Spindexer spindexer,
                                          int ballCount, double spinupTimeout) {
        return new SequentialCommandGroup(
            new WaitForShooterReadyCommand(shooter, spinupTimeout),
            shootAllBalls(shooter, spindexer, ballCount)
        );
    }

    /**
     * Shoots a single ball.
     * Sequence: Fire -> Flick
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @return a command that shoots one ball
     */
    public static Command shootSingleBall(Shooter shooter, Spindexer spindexer) {
        return new SequentialCommandGroup(
            new FireCommand(shooter),
            new FlickCommand(spindexer),
            new SpindexerResetCommand(spindexer)
        );
    }

    /**
     * Shoots a single ball then rotates to next.
     * Useful for shooting in a loop with external control.
     *
     * @param shooter the shooter subsystem
     * @param spindexer the spindexer subsystem
     * @return a command that shoots one ball and prepares the next
     */
    public static Command shootAndAdvance(Shooter shooter, Spindexer spindexer) {
        return new SequentialCommandGroup(
            new FireCommand(shooter),
            new FlickCommand(spindexer),
            new RotateToNextCommand(spindexer),
            new SpindexerResetCommand(spindexer)
        );
    }
}
