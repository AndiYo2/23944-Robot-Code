package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import Constants.EnumConstants.BallColor;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

/**
 * Command that sets the initial spindexer ball pattern without cataloging.
 * Use this to skip the cataloging step when ball positions are known at match start.
 * Completes immediately.
 */
public class PreloadCommand extends InstantCommand {

    /**
     * Creates a preload command with the default pattern.
     */
    public PreloadCommand() {
        super(() -> SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                SpindexerConstants.DEFAULT_PRELOAD[0],
                SpindexerConstants.DEFAULT_PRELOAD[1],
                SpindexerConstants.DEFAULT_PRELOAD[2]
        ));
    }

    /**
     * Creates a preload command with a custom pattern.
     *
     * @param pattern array of 3 BallColors: [Intake, Shooter, TopStorage]
     */
    public PreloadCommand(BallColor[] pattern) {
        super(() -> SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                pattern[0], pattern[1], pattern[2]
        ));
    }

    /**
     * Creates a preload command with explicit ball colors.
     *
     * @param intake ball color in intake slot (slot 0)
     * @param shooter ball color in shooter slot (slot 1)
     * @param topStorage ball color in top storage slot (slot 2)
     */
    public PreloadCommand(BallColor intake, BallColor shooter, BallColor topStorage) {
        super(() -> SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                intake, shooter, topStorage
        ));
    }
}
