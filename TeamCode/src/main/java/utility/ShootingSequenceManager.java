package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import subsystems.Shooter;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.RobotConstants;
import Constants.SpindexerConstants;

import static utility.SpindexerAndMotifStatus.*;

/**
 * ShootingSequenceManager - Simple 3-ball shooting sequence
 *
 * Shoots 3 balls with this sequence for each:
 * 1. Activate spindexer flipper
 * 2. Wait SHOOTER_FLIPPER_TIME (0.02s)
 * 3. Remove ball from slot in code
 * 4. Rotate spindexer to nearest correct color in pattern
 * 5. Wait EXTRA_WAIT_TIME (0.15s)
 * 6. Flip shooter flipper
 * 7. Wait for rotation complete, then repeat
 */
public class ShootingSequenceManager {
    private final Spindexer spindexer;
    private final Shooter shooter;

    private enum State {
        IDLE,
        FLICK_SPINDEXER,
        WAIT_FLIPPER_TIME,
        REMOVE_BALL_AND_START_ROTATION,
        WAIT_EXTRA,
        FLICK_SHOOTER,
        WAIT_ROTATION
    }

    private State state = State.IDLE;
    private int ballsShot = 0;
    private ElapsedTime timer = new ElapsedTime();

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
        state = State.WAIT_ROTATION;
        spindexer.rotateToColor(MotifPattern.getBallColorInSlotX(0));
    }

    /**
     * Update state machine - call every loop
     */
    public void update() {
        switch (state) {
            case IDLE:
                return;

            case FLICK_SPINDEXER:
                // Wait for spindexer flipper to extend
                if (spindexer.getCurrentState() == FlickState.Extended) {
                    timer.reset();
                    state = State.WAIT_FLIPPER_TIME;
                }
                break;

            case WAIT_FLIPPER_TIME:
                // Wait SHOOTER_FLIPPER_TIME (0.02s)
                if (timer.seconds() >= RobotConstants.ShootingSequence.SHOOTER_FLIPPER_TIME) {
                    state = State.REMOVE_BALL_AND_START_ROTATION;
                }
                break;

            case REMOVE_BALL_AND_START_ROTATION:
                // Remove ball from slot in code
                SpindexerConstants.spindexerPattern.setBallPatternNone(1);
                ballsShot++;

                // Start rotation to nearest correct color (if not last ball)
                if (ballsShot < RobotConstants.ShootingSequence.TOTAL_BALLS) {
                    spindexer.rotateToColor(MotifPattern.getBallColorInSlotX(ballsShot));
                }

                timer.reset();
                state = State.WAIT_EXTRA;
                break;

            case WAIT_EXTRA:
                // Wait extra 0.075s
                if (timer.seconds() >= RobotConstants.ShootingSequence.EXTRA_WAIT_TIME) {
                    shooter.triggerShot();
                    state = State.FLICK_SHOOTER;
                }
                break;

            case FLICK_SHOOTER:
                // Wait for shooter flipper to complete, then check if we need to repeat
                if (shooter.getCurrentState() == FlickState.Idle) {
                    if (ballsShot >= RobotConstants.ShootingSequence.TOTAL_BALLS) {
                        // Done with all balls
                        state = State.IDLE;
                    } else {
                        // Wait for spindexer rotation to complete
                        state = State.WAIT_ROTATION;
                    }
                }
                break;

            case WAIT_ROTATION:
                // Wait for spindexer to finish rotating
                if (spindexer.isReadyToFlip()) {
                    // Rotation complete, start next ball
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
                RobotConstants.ShootingSequence.TOTAL_BALLS
        );
    }

    public int getBallsShot() {
        return ballsShot;
    }
}