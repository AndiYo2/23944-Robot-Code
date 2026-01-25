package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import Constants.EnumConstants;
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
            return sequence;
        }
        sequence.addCommands(new FireCommand(shooter));

        if(ballCount == 2){
            sequence.addCommands(new FlickCommand(spindexer));
            sequence.addCommands(new FireCommand(shooter));
        }else if (ballCount == 3){
            sequence.addCommands(new FlickCommand(spindexer));
            sequence.addCommands(new RotateToNextCommand(spindexer).alongWith(new FireCommand(shooter)));
            sequence.addCommands(new FlickCommand(spindexer));
            sequence.addCommands(new FireCommand(shooter));
        }
        sequence.addCommands(new SpindexerResetCommand(spindexer));
        return sequence;
    }

}
