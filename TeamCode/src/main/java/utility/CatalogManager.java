package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants.Enums.BallColor;

/**
 * CatalogManager
 *
 * Manages automatic ball cataloging and filling sequence.
 * When intake is released, automatically fills spindexer slots sequentially.
 *
 * Sequence:
 * 1. When intake stops, if ball present, catalog it and rotate to nearest empty slot
 * 2. After rotation completes, automatically run intake
 * 3. Wait for ball to enter (with timeout)
 * 4. If ball enters, repeat sequence
 * 5. If timeout or spindexer full, end cycle
 *
 * Usage:
 * - Call update(intakeActive) in your TeleOp/Auto loop
 */
public class CatalogManager {
    private final DualColorSensor intakeSensor;
    private final Spindexer spindexer;
    private final Intake intake;

    // State machine states
    private enum CatalogState {
        IDLE,                   // Waiting for user to release intake
        CATALOG_AND_ROTATE,     // Cataloging ball and initiating rotation
        WAITING_FOR_ROTATION,   // Spindexer rotating to next empty slot
        AUTO_INTAKE,            // Automatically running intake after rotation
        WAITING_FOR_BALL,       // Waiting for ball to enter (with timeout)
        DONE                    // Cycle complete (full or timeout)
    }

    private CatalogState currentState = CatalogState.IDLE;
    private boolean lastIntakeState = false;
    private boolean ballDetectedFlag = false;
    private ElapsedTime ballWaitTimer = new ElapsedTime();
    private int currentIntakeSlot = 0; // Tracks which slot (0, 1, or 2) is currently at the intake position
    private BallColor pendingBallColor = BallColor.None; // Store detected color to avoid double-scan issues

    // Configuration
    private static final double BALL_WAIT_TIMEOUT = 2.0; // seconds to wait for ball
    private static final double BALL_DETECTION_COOLDOWN = 0.3; // seconds between detections
    private static final double INTAKE_START_DELAY = 0.5; // seconds to wait before starting ball detection

    public CatalogManager(DualColorSensor intakeSensor, Spindexer spindexer, Intake intake) {
        this.intakeSensor = intakeSensor;
        this.spindexer = spindexer;
        this.intake = intake;
    }

    /**
     * Updates the catalog manager state machine. Call this in your periodic loop.
     * @param intakeActive true if user is holding intake trigger, false otherwise
     */
    public void update(boolean intakeActive) {
        // Detect intake trigger release (transition from active to inactive)
        boolean intakeJustReleased = lastIntakeState && !intakeActive;
        boolean intakeJustPressed = !lastIntakeState && intakeActive;
        lastIntakeState = intakeActive;

        // If user presses trigger during auto-cataloging, abort and return to manual control
        if (intakeJustPressed && isCataloging()) {
            reset();
            return;
        }

        // If user manually starts intake while in DONE state, reset to IDLE
        if (currentState == CatalogState.DONE && intakeActive) {
            currentState = CatalogState.IDLE;
        }

        // State machine
        switch (currentState) {
            case IDLE:
                // User is manually controlling intake
                // When they release the trigger, check if there's a ball to catalog
                if (intakeJustReleased) {
                    intakeSensor.refreshScan();
                    BallColor detectedColor = intakeSensor.getBallColor();

                    if (detectedColor != BallColor.None) {
                        // Ball present - save it and start catalog sequence
                        pendingBallColor = detectedColor;
                        currentState = CatalogState.CATALOG_AND_ROTATE;
                    }
                }
                break;

            case CATALOG_AND_ROTATE:
                // Check if spindexer is full
                if (spindexer.isFull()) {
                    // Stop the cycle - spindexer is full
                    intake.stopIntake();
                    currentState = CatalogState.DONE;
                    pendingBallColor = BallColor.None;
                    break;
                }

                // Use the pendingBallColor from previous detection
                // DON'T re-scan here to avoid missing the ball due to sensor flicker
                if (pendingBallColor != BallColor.None) {
                    // Catalog ball at whichever slot is currently at the intake position
                    spindexer.catalogBall(currentIntakeSlot, pendingBallColor);

                    // Always rotate forward 120° to next slot
                    spindexer.rotateToNextSlot();

                    // Update which slot is now at intake (cycles: 0→1→2→0)
                    currentIntakeSlot = (currentIntakeSlot + 1) % 3;

                    // Clear pending ball
                    pendingBallColor = BallColor.None;

                    currentState = CatalogState.WAITING_FOR_ROTATION;
                } else {
                    // No ball detected - end cycle
                    intake.stopIntake();
                    currentState = CatalogState.DONE;
                }
                break;

            case WAITING_FOR_ROTATION:
                // Wait for spindexer to finish rotating
                if (spindexer.isDoneRotating()) {
                    // Rotation complete - automatically start intake
                    currentState = CatalogState.AUTO_INTAKE;
                }
                break;

            case AUTO_INTAKE:
                // Start intake and begin waiting for ball
                intake.runIntake();
                ballWaitTimer.reset();
                ballDetectedFlag = false;
                currentState = CatalogState.WAITING_FOR_BALL;
                break;

            case WAITING_FOR_BALL:
                // Check for timeout
                if (ballWaitTimer.seconds() >= BALL_WAIT_TIMEOUT) {
                    // Timeout - no ball entered, stop intake and end cycle
                    intake.stopIntake();
                    currentState = CatalogState.DONE;
                    break;
                }

                // Check if spindexer became full (shouldn't happen, but safety check)
                if (spindexer.isFull()) {
                    intake.stopIntake();
                    currentState = CatalogState.DONE;
                    break;
                }

                // Wait for intake to stabilize before starting ball detection
                if (ballWaitTimer.seconds() < INTAKE_START_DELAY) {
                    break; // Still in startup delay
                }

                // Scan for ball
                intakeSensor.refreshScan();
                BallColor currentColor = intakeSensor.getBallColor();
                boolean ballPresent = currentColor != BallColor.None;

                // Detect ball entry (transition from no ball to ball present)
                if (ballPresent && !ballDetectedFlag && ballWaitTimer.seconds() > BALL_DETECTION_COOLDOWN) {
                    // Ball detected! Save color, stop intake, and catalog it
                    pendingBallColor = currentColor;
                    intake.stopIntake();
                    currentState = CatalogState.CATALOG_AND_ROTATE;
                }

                ballDetectedFlag = ballPresent;
                break;

            case DONE:
                // Cycle complete - wait for user to manually start intake again
                // (transitions back to IDLE when user presses intake trigger)
                break;
        }
    }

    /**
     * Resets the catalog manager state.
     * Call this when you want to abort any in-progress cataloging.
     */
    public void reset() {
        currentState = CatalogState.IDLE;
        ballDetectedFlag = false;
        currentIntakeSlot = 0;
        pendingBallColor = BallColor.None;
        intake.stopIntake();
    }

    /**
     * Returns true if auto-fill cycle is running
     */
    public boolean isCataloging() {
        return currentState != CatalogState.IDLE && currentState != CatalogState.DONE;
    }

    /**
     * Gets status string for telemetry
     */
    public String getStatus() {
        switch (currentState) {
            case IDLE:
                return String.format("IDLE - Next slot: %d", currentIntakeSlot);
            case CATALOG_AND_ROTATE:
                return String.format("Cataloging slot %d", currentIntakeSlot);
            case WAITING_FOR_ROTATION:
                return "Rotating spindexer";
            case AUTO_INTAKE:
                return "Starting intake";
            case WAITING_FOR_BALL:
                return String.format("Waiting for ball in slot %d (%.1fs)",
                    currentIntakeSlot, BALL_WAIT_TIMEOUT - ballWaitTimer.seconds());
            case DONE:
                return spindexer.isFull() ? "FULL - All slots filled" : "DONE - Timeout";
            default:
                return "UNKNOWN";
        }
    }

    /**
     * Gets current state (for debugging)
     */
    public String getCurrentState() {
        return currentState.toString();
    }

    /**
     * Gets which slot is currently at the intake position
     */
    public int getCurrentIntakeSlot() {
        return currentIntakeSlot;
    }
}
