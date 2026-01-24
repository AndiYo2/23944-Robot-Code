package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import Constants.SpindexerConstants;
import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.ShootingMode;

/**
 * Rotates the spindexer to the next ball position.
 * In Fast mode: rotates to nearest ball.
 * In Sorted mode: rotates to specific color based on motif pattern.
 */
public class RotateAction implements SpindexerAction {
    private final Spindexer spindexer;
    private final ShootingMode mode;
    private final BallColor targetColor;
    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Rotate to next closest ball (Fast mode).
     */
    public RotateAction(Spindexer spindexer) {
        this.spindexer = spindexer;
        this.mode = ShootingMode.Fast;
        this.targetColor = null;
    }

    /**
     * Rotate to a specific color (Sorted mode).
     */
    public RotateAction(Spindexer spindexer, BallColor targetColor) {
        this.spindexer = spindexer;
        this.mode = ShootingMode.Sorted;
        this.targetColor = targetColor;
    }

    @Override
    public void start() {
        if (mode == ShootingMode.Sorted && targetColor != null) {
            spindexer.rotateToColor(targetColor);
        } else {
            spindexer.rotateToNextClosestBall();
        }
        timer.reset();
    }

    @Override
    public void update() {}

    @Override
    public boolean isComplete() {
        return timer.seconds() >= SpindexerConstants.ROTATION_TIME;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        if (targetColor != null) {
            return "Rotate[" + targetColor + "]";
        }
        return "Rotate[Next]";
    }
}
