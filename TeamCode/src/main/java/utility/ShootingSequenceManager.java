package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Spindexer;
import subsystems.Shooter;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.EnumConstants.ShootingMode;
import Constants.RobotConstants;

import static utility.SpindexerAndMotifStatus.*;

/**
 * ShootingSequenceManager - Dual-mode shooting sequence controller
 *
 * Supports two modes:
 * - Fast: Pipeline approach, one trigger = one ball, auto-preps next
 * - Sorted: One trigger = all balls shot in motif pattern order
 */
public class ShootingSequenceManager {
    private final Spindexer spindexer;
    private final Shooter shooter;

    private enum State {
        IDLE,                    // No sequence active
        CHECK_SHOOTER_SLOT,      // Check if ball already in slot 1
        PREPARING,               // Rotating to target ball
        READY_TO_SHOOT,          // Ball in shooter, waiting for trigger (Fast mode only)
        FLICK_SPINDEXER,         // Spindexer flipper extending
        WAIT_SPINDEXER_RETRACT,  // Waiting for spindexer flipper to retract
        FLICK_SHOOTER,           // Shooter flipper launching ball
        WAIT_SHOOTER_IDLE,       // Waiting for shooter flipper to complete
        ROTATING_TO_NEXT,        // Rotating to next ball
        RESETTING                // Moving back to 300 degrees
    }

    private State state = State.IDLE;
    private ShootingMode currentMode = ShootingMode.Fast;
    private int ballsShot = 0;
    private int initialBallCount = 0;
    private ElapsedTime timer = new ElapsedTime();

    public ShootingSequenceManager(Spindexer spindexer, Shooter shooter) {
        this.spindexer = spindexer;
        this.shooter = shooter;
    }

    // ==================== MODE CONTROL ====================

    public void setMode(ShootingMode mode) {
        this.currentMode = mode;
    }

    public ShootingMode getMode() {
        return currentMode;
    }

    public void toggleMode() {
        if (state != State.IDLE) return; // Don't toggle during sequence
        currentMode = (currentMode == ShootingMode.Fast) ? ShootingMode.Sorted : ShootingMode.Fast;
    }

    // ==================== SEQUENCE CONTROL ====================

    /**
     * Start the shooting sequence based on current mode.
     * For Fast mode: Preps first ball and waits for trigger.
     * For Sorted mode: Shoots all balls automatically.
     */
    public void startSequence() {
        if (state != State.IDLE) return;

        // Check if we have any balls
        initialBallCount = SpindexerPattern.getBallCount();
        if (initialBallCount == 0) {
            // No balls - just reset to 300
            state = State.RESETTING;
            return;
        }

        ballsShot = 0;
        state = State.CHECK_SHOOTER_SLOT;
    }

    /**
     * For Fast mode: Triggers a shot when in READY_TO_SHOOT state.
     * For Sorted mode: This is called automatically, not by user.
     */
    public void triggerShot() {
        if (currentMode == ShootingMode.Fast && state == State.READY_TO_SHOOT) {
            state = State.FLICK_SHOOTER;
            shooter.triggerShot();
        }
    }

    // ==================== STATE MACHINE UPDATE ====================

    public void update() {
        switch (state) {
            case IDLE:
                return;

            case CHECK_SHOOTER_SLOT:
                handleCheckShooterSlot();
                break;

            case PREPARING:
                handlePreparing();
                break;

            case READY_TO_SHOOT:
                // Fast mode: waiting for triggerShot() call
                // Sorted mode: should never stay here, transitions immediately
                break;

            case FLICK_SPINDEXER:
                handleFlickSpindexer();
                break;

            case WAIT_SPINDEXER_RETRACT:
                handleWaitSpindexerRetract();
                break;

            case FLICK_SHOOTER:
                handleFlickShooter();
                break;

            case WAIT_SHOOTER_IDLE:
                handleWaitShooterIdle();
                break;

            case ROTATING_TO_NEXT:
                handleRotatingToNext();
                break;

            case RESETTING:
                handleResetting();
                break;
        }
    }

    // ==================== STATE HANDLERS ====================

    private void handleCheckShooterSlot() {
        EnumConstants.BallColor slot1Ball = SpindexerPattern.getBallInSlotX(1);

        if (slot1Ball != EnumConstants.BallColor.None) {
            // Ball in slot 1 - flick it into shooter
            state = State.FLICK_SPINDEXER;
            spindexer.triggerFlick();
            // Remove ball from tracking when flicking
            SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
        } else {
            // No ball in slot 1 - need to rotate one in
            state = State.PREPARING;
            rotateToTargetBall();
        }
    }

    private void handlePreparing() {
        // Wait for rotation to complete
        if (!spindexer.isRotationIdle()) return;

        timer.reset();
        // Check if we now have a ball in slot 1
        if (SpindexerPattern.getBallInSlotX(1) != EnumConstants.BallColor.None) {
            // Ball rotated in - flick it
            state = State.FLICK_SPINDEXER;
            spindexer.triggerFlick();
            // Remove ball from tracking when flicking
            SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
        } else {
            // No more balls - reset
            state = State.RESETTING;
        }
    }

    private void handleFlickSpindexer() {
        // Wait for spindexer flipper to extend (or already done)
        FlickState spindexerState = spindexer.getCurrentState();
        if (spindexerState == FlickState.Extended ||
            spindexerState == FlickState.Retracted ||
            spindexerState == FlickState.Idle) {
            timer.reset();
            state = State.WAIT_SPINDEXER_RETRACT;
        }
    }

    private void handleWaitSpindexerRetract() {
        // Wait for spindexer flipper to retract
        if (spindexer.getCurrentState() == FlickState.Retracted ||
            spindexer.getCurrentState() == FlickState.Idle) {

            // Wait settle time before shooting
            if (timer.seconds() < RobotConstants.ShootingSequenceV2.POST_FLICK_SETTLE_TIME) return;

            // Ball is now in shooter - shoot it
            if (currentMode == ShootingMode.Fast && ballsShot > 0) {
                // Fast mode (subsequent balls): wait for trigger
                state = State.READY_TO_SHOOT;
            } else {
                // First ball in Fast mode OR Sorted mode: immediately trigger shot
                state = State.FLICK_SHOOTER;
                shooter.triggerShot();
            }
        }
    }

    private void handleFlickShooter() {
        FlickState shooterState = shooter.getCurrentState();

        // If shooter is still Idle, triggerShot() didn't work - try again
        if (shooterState == FlickState.Idle) {
            shooter.triggerShot();
            return;
        }

        // Wait for shooter flipper to extend (or already past extended)
        if (shooterState == FlickState.Extended ||
            shooterState == FlickState.Retracted) {
            timer.reset();
            state = State.WAIT_SHOOTER_IDLE;
        }
        // If still in Start state, wait for periodic() to process it
    }

    private void handleWaitShooterIdle() {
        // Wait for shooter flipper to complete
        if (shooter.getCurrentState() != FlickState.Idle) return;

        // Wait settle time
        if (timer.seconds() < RobotConstants.ShootingSequenceV2.POST_SHOT_SETTLE_TIME) return;

        ballsShot++;

        // Check if more balls remain
        if (SpindexerPattern.getBallCount() > 0) {
            // More balls - rotate to get next ball into slot 1
            state = State.ROTATING_TO_NEXT;
            timer.reset();
            rotateToTargetBall();
        } else {
            // No more balls - reset to 300
            state = State.RESETTING;
        }
    }

    private void handleRotatingToNext() {
        // Wait for rotation to complete
        if (!spindexer.isRotationIdle()) return;

        // Wait settle time
        if (timer.seconds() < RobotConstants.ShootingSequenceV2.POST_ROTATION_SETTLE_TIME) return;

        // Check if we have a ball in slot 1 to push into shooter
        if (SpindexerPattern.getBallInSlotX(1) != EnumConstants.BallColor.None) {
            // Flick spindexer to push ball into shooter
            state = State.FLICK_SPINDEXER;
            spindexer.triggerFlick();
            // Remove ball from tracking when flicking
            SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
        } else {
            // No ball to push - reset
            state = State.RESETTING;
        }
    }

    private void handleResetting() {
        // Reset spindexer to 300 degrees
        spindexer.resetToEmptyPosition();
        state = State.IDLE;
    }

    // ==================== ROTATION HELPERS ====================

    /**
     * Rotate to the target ball based on current mode.
     * Fast mode: Rotate to nearest ball
     * Sorted mode: Rotate to motif pattern color
     */
    private void rotateToTargetBall() {
        if (currentMode == ShootingMode.Sorted) {
            // Sorted mode: try to rotate to the correct color in motif pattern
            EnumConstants.BallColor targetColor = MotifPattern.getBallColorInSlotX(ballsShot);
            spindexer.rotateToColor(targetColor);
            // rotateToColor returns false if color not found, but it still rotates to nearest ball
        } else {
            // Fast mode: rotate to nearest ball
            spindexer.rotateToNextClosestBall();
        }
    }

    // ==================== STATUS METHODS ====================

    public boolean isIdle() {
        return state == State.IDLE;
    }

    public boolean isExecuting() {
        return state != State.IDLE;
    }

    public boolean isReadyToShoot() {
        return state == State.READY_TO_SHOOT;
    }

    public State getState() {
        return state;
    }

    public String getStatus() {
        if (state == State.IDLE) {
            return String.format("IDLE [%s]", currentMode);
        }
        return String.format("%s [%s] - Ball %d/%d",
                state.toString(),
                currentMode,
                ballsShot + 1,
                initialBallCount
        );
    }

    public int getBallsShot() {
        return ballsShot;
    }
}
