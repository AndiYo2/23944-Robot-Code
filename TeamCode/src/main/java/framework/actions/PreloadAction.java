package framework.actions;

import framework.Action;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

/**
 * Action that sets the initial spindexer ball pattern without cataloging.
 * Use this to skip the cataloging step when ball positions are known at match start.
 * Completes immediately in one cycle.
 */
public class PreloadAction implements Action {
    private final EnumConstants.BallColor[] pattern;

    /**
     * Creates a preload action with the default pattern (Purple, Purple, Green).
     */
    public PreloadAction() {
        this.pattern = SpindexerConstants.DEFAULT_PRELOAD;
    }

    /**
     * Creates a preload action with a custom pattern.
     *
     * @param pattern array of 3 BallColors: [Intake, Shooter, TopStorage]
     */
    public PreloadAction(EnumConstants.BallColor[] pattern) {
        this.pattern = pattern;
    }

    /**
     * Creates a preload action with explicit ball colors.
     *
     * @param intake ball color in intake slot (slot 0)
     * @param shooter ball color in shooter slot (slot 1)
     * @param topStorage ball color in top storage slot (slot 2)
     */
    public PreloadAction(EnumConstants.BallColor intake,
                         EnumConstants.BallColor shooter,
                         EnumConstants.BallColor topStorage) {
        this.pattern = new EnumConstants.BallColor[]{intake, shooter, topStorage};
    }

    @Override
    public void start() {
        SpindexerAndMotifStatus.SpindexerPattern.setBallPattern(
                pattern[0], pattern[1], pattern[2]);
    }

    @Override
    public void update() {
        // Immediate action, nothing to update
    }

    @Override
    public boolean isComplete() {
        // Completes immediately after start()
        return true;
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return String.format("Preload[%s,%s,%s]", pattern[0], pattern[1], pattern[2]);
    }
}
