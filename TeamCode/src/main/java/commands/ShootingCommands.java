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
     * Shoots 3 balls.
     * Cataloging always ends at [m2, m1, None], so this uses a fixed sequence.
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
            // Cataloging always ends at [m2, m1, None], so shooting is fixed:
            // fire m0 (above), flick m1, CCW (m2 from slot 0 → slot 1), fire m1, flick m2, fire m2
            sequence.addCommands(new FlickCommand(spindexer));
            sequence.addCommands(new RotateCCWCommand(spindexer).alongWith(new FireCommand(shooter)));
            sequence.addCommands(new FlickCommand(spindexer));
            sequence.addCommands(new FireCommand(shooter));
        }
        sequence.addCommands(new SpindexerResetCommand(spindexer));
        return sequence;
    }

}
