package utility.Shooting;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import subsystems.Shooter;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;

import static utility.SpindexerAndMotifStatus.*;

/**
 * ShootingSequenceManager - Simple 3-ball shooting sequence
 *
 * Shoots 3 balls with this sequence for each:
 * 1. Flick spindexer out
 * 2. Wait 0.2s
 * 3. Flick shooter out
 * 4. Wait 0.1s
 * 5. Rotate spindexer (except on 3rd ball)
 * 6. Wait for rotation complete
 * 7. Repeat
 */
public class ShootingSequenceManager {
    private final Spindexer spindexer;
    private final Shooter shooter;

    private enum State {
        IDLE,
        FLICK_SPINDEXER,
        WAIT_BEFORE_SHOOTER,
        FLICK_SHOOTER,
        WAIT_AFTER_SHOOTER,
        ROTATE_SPINDEXER,
        WAIT_ROTATION,
        CLEANUP
    }

    private State state = State.IDLE;
    private int ballsShot = 0;
    private static final int TOTAL_BALLS = 3;
    private ElapsedTime timer = new ElapsedTime();

    // Timing constants
    private static final double DELAY_BEFORE_SHOOTER = 0.02;  // 200ms between spindexer and shooter

    public ShootingSequenceManager(Spindexer spindexer, Shooter shooter) {
        this.spindexer = spindexer;
        this.shooter = shooter;
    }

    /**
     * Start shooting 3 balls
     */
    public void startShootingSequence() {
        if (state != State.IDLE) {
            return; // Already running
        }

        // Only start if spindexer is ready (not rotating, at rest)
        if (!spindexer.isReadyToFlip()) {
            return; // Wait until spindexer is at rest
        }

        ballsShot = 0;
        state = State.FLICK_SPINDEXER;
        spindexer.triggerFlick();
    }

    /**
     * Update state machine - call every loop
     */
    public void update() {
        switch (state) {
            case IDLE:
                return;
            case FLICK_SPINDEXER:
                // Wait for spindexer to extend
                if (spindexer.getCurrentState() == FlickState.Extended) {
                    timer.reset();
                    state = State.WAIT_BEFORE_SHOOTER;
                }
                break;

            case WAIT_BEFORE_SHOOTER:
                // Wait 0.2s
                if (timer.seconds() >= DELAY_BEFORE_SHOOTER) {
                    shooter.triggerShot();
                    state = State.CLEANUP;
                }
                break;
            case CLEANUP:
                // Clear ball from pattern
                RobotConstants.Spindexer.spindexerPattern.setBallPatternNone(1);
                ballsShot++;

                // Check if done with all 3 balls
                if (ballsShot >= TOTAL_BALLS) {
                    state = State.IDLE;
                } else {
                    // More balls to shoot - rotate spindexer
                    state = State.ROTATE_SPINDEXER;
                    spindexer.rotateToColor( MotifPattern.getBallColorInSlotX(ballsShot));
                }
                break;
            case ROTATE_SPINDEXER:
                // Rotation command sent, move to waiting
                state = State.WAIT_ROTATION;
                break;
            case WAIT_ROTATION:
                // Wait for spindexer to finish rotating (state = IDLE, regardless of position accuracy)
                if (spindexer.isReadyToFlip()) {
                    // Rotation attempt complete, shoot next ball
                    state = State.FLICK_SPINDEXER;
                    spindexer.triggerFlick();
                }
                break;
        }
    }

    public boolean isIdle() {
        return state == State.IDLE;
    }

    public boolean isExecuting() {
        return state != State.IDLE;
    }

    public String getStatus() {
        if (state == State.IDLE) {
            return "IDLE";
        }
        return String.format("%s - Ball %d/%d",
            state.toString(),
            ballsShot + 1,
            TOTAL_BALLS
        );
    }

    public int getBallsShot() {
        return ballsShot;
    }
}
