package utility;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants.Enums.BallColor;

/**
 * CatalogManager
 *
 * Manages automatic ball cataloging when intake is released.
 * Detects ball color, stores it in the spindexer pattern, and rotates the spindexer.
 * If the spindexer is full (3 balls), it reverses the intake to expel extra balls.
 *
 * Usage:
 * - Call update(intakeActive) in your TeleOp/Auto loop
 * - When intake is released, it will automatically catalog and rotate
 */
public class CatalogManager {
    private final DualColorSensor intakeSensor;
    private final Spindexer spindexer;
    private final Intake intake;

    private boolean lastIntakeState = false;
    private boolean catalogInProgress = false;
    private boolean reverseInProgress = false;
    private ElapsedTime reverseTimer = new ElapsedTime();

    // Configuration
    private static final double INTAKE_REVERSE_DURATION = 2.0; // seconds
    private static final int INTAKE_SLOT_INDEX = 0; // Slot 0 is the intake position

    public CatalogManager(DualColorSensor intakeSensor, Spindexer spindexer, Intake intake) {
        this.intakeSensor = intakeSensor;
        this.spindexer = spindexer;
        this.intake = intake;
    }

    /**
     * Updates the catalog manager. Call this in your periodic loop.
     * @param intakeActive true if intake is currently running, false otherwise
     */
    public void update(boolean intakeActive) {
        // Handle reverse timeout
        if (reverseInProgress) {
            if (reverseTimer.seconds() >= INTAKE_REVERSE_DURATION) {
                intake.stopIntake();
                reverseInProgress = false;
            }
            return; // Don't catalog while reversing
        }

        // Detect intake release (was running, now stopped)
        if (lastIntakeState && !intakeActive && !catalogInProgress) {
            catalogBall();
        }

        lastIntakeState = intakeActive;

        // Reset catalog flag when rotation is complete
        if (catalogInProgress && spindexer.isDoneRotating()) {
            catalogInProgress = false;
        }
    }

    /**
     * Catalogs the ball currently in the intake slot.
     * If spindexer is full, reverses intake to expel extra balls.
     * Otherwise, catalogs the ball and rotates the spindexer 120 degrees.
     */
    private void catalogBall() {
        // Check if spindexer is full (3 balls)
        if (spindexer.isFull()) {
            // Reverse intake to expel extra balls
            intake.reverseIntake();
            reverseInProgress = true;
            reverseTimer.reset();
            return;
        }

        // Scan ball color
        intakeSensor.refreshScan();
        BallColor detectedColor = intakeSensor.getBallColor();

        // Only catalog if we detected an actual ball (not None)
        if (detectedColor != BallColor.None) {
            // Add to spindexer catalog at intake slot (slot 0)
            spindexer.catalogBall(INTAKE_SLOT_INDEX, detectedColor);

            // Rotate 120 degrees forward to next slot
            spindexer.rotateToNextSlot();
            catalogInProgress = true;
        }
    }

    /**
     * Resets the catalog manager state.
     * Call this when you want to clear any in-progress cataloging.
     */
    public void reset() {
        catalogInProgress = false;
        reverseInProgress = false;
        lastIntakeState = false;
    }

    /**
     * Returns true if a catalog operation is currently in progress (rotating)
     */
    public boolean isCataloging() {
        return catalogInProgress;
    }

    /**
     * Returns true if intake is currently reversing due to full spindexer
     */
    public boolean isReversing() {
        return reverseInProgress;
    }

    /**
     * Gets status string for telemetry
     */
    public String getStatus() {
        if (reverseInProgress) {
            return String.format("REVERSING (%.1fs remaining)",
                INTAKE_REVERSE_DURATION - reverseTimer.seconds());
        } else if (catalogInProgress) {
            return "CATALOGING (rotating)";
        } else {
            return "READY";
        }
    }
}
