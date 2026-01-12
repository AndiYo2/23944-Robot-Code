package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

import static Constants.SpindexerConstants.*;

/**
 * Spindexer subsystem - Position-controlled servo mode with encoder verification.
 *
 * Uses 6 servo positions (0.0, 0.2, 0.4, 0.6, 0.8, 1.0) mapped to 3 physical slots.
 * Each slot has two equivalent positions due to 2:1 gear ratio:
 *   - Slot 0: positions 0.0 and 0.6
 *   - Slot 1: positions 0.2 and 0.8
 *   - Slot 2: positions 0.4 and 1.0
 *
 * Boundary wrapping (safety net only - should not occur in normal operation):
 *   - At 1.0, CW wraps to 0.6
 *   - At 0.0, CCW wraps to 0.4
 */
public class Spindexer implements Subsystem {

    // Hardware reference
    private final RobotHardware robot;

    // State tracking
    private RotationState rotationState = RotationState.IDLE;
    private FlickState currentState = FlickState.Idle;

    // Extended rotation states for verification and retry
    public enum RotationState {
        IDLE,
        ROTATING,
        VERIFYING,
        RETRYING,
        ERROR
    }

    // Position tracking
    private double currentServoPosition = EMPTY_RESET_POSITION;
    private double lastCommandedPosition = EMPTY_RESET_POSITION;
    private boolean isWrapRotation = false;

    // Retry tracking
    private int retryCount = 0;

    // Timers
    private final ElapsedTime flickerTimer = new ElapsedTime();
    private final ElapsedTime rotationTimer = new ElapsedTime();
    private final ElapsedTime verificationTimer = new ElapsedTime();

    // Initialization state
    private boolean needsInitialization = true;
    private double initialEncoderVoltage = -1;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();

        // Initialize flipper to retracted position
        robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_RETRACT);

        // Initialize spindexer to position 0
        currentServoPosition = EMPTY_RESET_POSITION;
        robot.spindexerServo.setPosition(currentServoPosition);
    }

    // ==================== ROTATION METHODS ====================

    public void rotateCW() {
        if (rotationState != RotationState.IDLE) return;

        double newPosition = currentServoPosition + POSITION_INCREMENT;
        isWrapRotation = false;

        // Boundary wrapping (safety net - should rarely happen)
        if (newPosition > 1.0) {
            newPosition = CW_WRAP_TO;  // 0.6
            isWrapRotation = true;
            logWrapWarning("CW", currentServoPosition, newPosition);
        }

        setServoPosition(newPosition);
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCW();
    }

    public void rotateCCW() {
        if (rotationState != RotationState.IDLE) return;

        double newPosition = currentServoPosition - POSITION_INCREMENT;
        isWrapRotation = false;

        // Boundary wrapping (safety net - should rarely happen)
        if (newPosition < 0.0) {
            newPosition = CCW_WRAP_TO;  // 0.4
            isWrapRotation = true;
            logWrapWarning("CCW", currentServoPosition, newPosition);
        }

        setServoPosition(newPosition);
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCCW();
    }

    /**
     * Rotate to a specific ball color, choosing direction to avoid boundary wrapping.
     */
    public boolean rotateToColor(EnumConstants.BallColor color) {
        if (rotationState != RotationState.IDLE) return false;

        // Check slot 1 first (already in shooter position)
        if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1) == color) {
            return true; // Already aligned - NO movement
        }

        boolean slot0Has = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) == color;
        boolean slot2Has = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) == color;

        if (slot0Has && slot2Has) {
            // BOTH slots have target color - choose direction AWAY from boundary
            rotateAwayFromBoundary();
            return true;
        }

        // Only one slot has target - must go that direction
        if (slot2Has) {
            rotateCW();  // 0.2 position increase
            return true;
        }
        if (slot0Has) {
            rotateCCW(); // 0.2 position decrease
            return true;
        }

        // Color not found - rotate to any ball
        rotateToNextClosestBall();
        return false;
    }

    /**
     * Smart direction choice to minimize wrapping.
     */
    private void rotateAwayFromBoundary() {
        if (currentServoPosition <= 0.2) {
            rotateCW();   // Near 0.0 boundary → go CW (toward higher positions)
        } else if (currentServoPosition >= 0.8) {
            rotateCCW();  // Near 1.0 boundary → go CCW (toward lower positions)
        } else {
            // Safe zone (0.4-0.6) - prefer toward 0.4 for next load cycle
            if (currentServoPosition > 0.4) {
                rotateCCW();
            } else {
                rotateCW();
            }
        }
    }

    public void rotateToNextClosestBall() {
        boolean slot0Has = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) != EnumConstants.BallColor.None;
        boolean slot2Has = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) != EnumConstants.BallColor.None;

        if (slot0Has && slot2Has) {
            rotateAwayFromBoundary();  // Smart choice
        } else if (slot0Has) {
            rotateCCW();
        } else if (slot2Has) {
            rotateCW();
        }
    }

    public void rotateToNearestEmptySlot() {
        if (rotationState != RotationState.IDLE) return;
        if (SpindexerAndMotifStatus.SpindexerPattern.isFull()) return;

        // Check each slot for empty
        boolean slot0Empty = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) == EnumConstants.BallColor.None;
        boolean slot1Empty = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1) == EnumConstants.BallColor.None;
        boolean slot2Empty = SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) == EnumConstants.BallColor.None;

        if (slot1Empty) return; // Already at empty slot

        if (slot0Empty && slot2Empty) {
            rotateAwayFromBoundary();
        } else if (slot0Empty) {
            rotateCCW();
        } else if (slot2Empty) {
            rotateCW();
        }
    }

    /**
     * Reset to empty position (0.0) and clear ball tracking.
     */
    public void resetToEmptyPosition() {
        if (rotationState != RotationState.IDLE) return;

        currentServoPosition = EMPTY_RESET_POSITION;
        robot.spindexerServo.setPosition(currentServoPosition);
        SpindexerAndMotifStatus.SpindexerPattern.clearAll();
    }

    // ==================== SERVO CONTROL ====================

    private void setServoPosition(double position) {
        currentServoPosition = position;
        lastCommandedPosition = position;
        robot.spindexerServo.setPosition(position);
        rotationState = RotationState.ROTATING;
        rotationTimer.reset();
        retryCount = 0;
    }

    // ==================== FLIPPER METHODS ====================

    public void triggerFlick() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }

    // ==================== STATE QUERIES ====================

    public boolean isRotationIdle() {
        return rotationState == RotationState.IDLE;
    }

    public RotationState getRotationState() {
        return rotationState;
    }

    public boolean isReadyToFlip() {
        return rotationState == RotationState.IDLE && currentState == FlickState.Idle;
    }

    public boolean isInError() {
        return rotationState == RotationState.ERROR;
    }

    public FlickState getCurrentState() {
        return currentState;
    }

    public double getServoPosition() {
        return currentServoPosition;
    }

    /**
     * Get current slot index (0, 1, or 2) based on servo position.
     * For backwards compatibility with telemetry.
     */
    public int getSpindPosTracker() {
        // Each 0.2 increment = 1 slot, wrapping at 3
        int positionIndex = (int) Math.round(currentServoPosition / POSITION_INCREMENT);
        return positionIndex % SLOTS_COUNT;
    }

    /**
     * Get target position in degrees (for telemetry compatibility).
     * Converts servo position (0-1) to approximate encoder degrees.
     */
    public int getTargetPosition() {
        return (int) servoPositionToExpectedDegrees(currentServoPosition);
    }

    /**
     * Check if rotation is complete (for backwards compatibility).
     */
    public boolean isDoneRotating() {
        return rotationState == RotationState.IDLE;
    }

    /**
     * Get current encoder position in degrees (for verification/telemetry).
     */
    public double getEncoderPositionDegrees() {
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= RobotConstants.Encoder.MAX_VOLTAGE;  // Normalize to 0-1
        pos *= RobotConstants.Encoder.FULL_ROTATION_DEGREES;  // Scale to degrees
        return pos;
    }

    /**
     * Convert servo position to expected encoder degrees.
     * With 2:1 gear ratio: servo position 0-1 = spindexer 0-600°
     */
    private double servoPositionToExpectedDegrees(double servoPos) {
        // servo 0-1 maps to 0-300° on servo, which is 0-600° on spindexer
        // But encoder reads spindexer directly (0-360° wrapping)
        double spindexerDegrees = servoPos * 600.0;  // 2:1 ratio
        return spindexerDegrees % 360.0;  // Wrap to encoder range
    }

    /**
     * Normalize angle difference to [-180, 180] range.
     */
    private double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    // ==================== PERIODIC UPDATES ====================

    @Override
    public void periodic() {
        // Lazy initialization - wait for encoder to be ready
        if (needsInitialization && isEncoderReady()) {
            initializeFromEncoder();
            needsInitialization = false;
        }

        flipperStateMachinePeriodic();
        rotationStateMachinePeriodic();

        // Auto-reset to position 0 when spindexer is empty
        checkAndResetIfEmpty();
    }

    /**
     * Automatically reset to position 0 when no balls are in the spindexer.
     * This ensures we start loading from position 0 for optimal sequencing.
     * Waits for both rotation and flipper to be idle before resetting.
     */
    private void checkAndResetIfEmpty() {
        // Don't reset while rotating or while flipper is active
        if (rotationState != RotationState.IDLE) return;
        if (currentState != FlickState.Idle) return;

        boolean isEmpty =
            SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) == EnumConstants.BallColor.None &&
            SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1) == EnumConstants.BallColor.None &&
            SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) == EnumConstants.BallColor.None;

        if (isEmpty && currentServoPosition != EMPTY_RESET_POSITION) {
            currentServoPosition = EMPTY_RESET_POSITION;
            robot.spindexerServo.setPosition(currentServoPosition);
        }
    }

    private void initializeFromEncoder() {
        double encoderDeg = getEncoderPositionDegrees();
        int nearestSlot = findNearestSlot(encoderDeg);
        // Snap to the lower equivalent position for the slot
        currentServoPosition = nearestSlot * POSITION_INCREMENT;
        robot.spindexerServo.setPosition(currentServoPosition);
    }

    private int findNearestSlot(double encoderDeg) {
        double minDistance = Double.MAX_VALUE;
        int nearestSlot = 0;

        for (int i = 0; i < SLOTS_COUNT; i++) {
            double slotDeg = SLOT_ENCODER_POSITIONS_DEG[i];
            double distance = Math.abs(normalizeAngle(encoderDeg - slotDeg));
            if (distance < minDistance) {
                minDistance = distance;
                nearestSlot = i;
            }
        }
        return nearestSlot;
    }

    private boolean isEncoderReady() {
        double currentVoltage = robot.spindexerEncoder.getVoltage();

        // First call - record initial voltage
        if (initialEncoderVoltage == -1) {
            initialEncoderVoltage = currentVoltage;
            return false;
        }

        // Check if voltage has changed from initial reading
        return Math.abs(currentVoltage - initialEncoderVoltage) > ENCODER_READY_THRESHOLD;
    }

    private void rotationStateMachinePeriodic() {
        switch (rotationState) {
            case IDLE:
                // Nothing to do
                break;

            case ROTATING:
                double timeout = isWrapRotation ? WRAP_ROTATION_TIME_MS : ROTATION_TIME_MS;
                if (rotationTimer.milliseconds() >= timeout) {
                    rotationState = RotationState.VERIFYING;
                    verificationTimer.reset();
                }
                break;

            case VERIFYING:
                double encoderDeg = getEncoderPositionDegrees();
                double expectedDeg = servoPositionToExpectedDegrees(currentServoPosition);
                double error = Math.abs(normalizeAngle(encoderDeg - expectedDeg));

                if (error < ENCODER_TOLERANCE_DEG) {
                    // Position verified successfully
                    rotationState = RotationState.IDLE;
                    retryCount = 0;
                } else if (verificationTimer.milliseconds() >= VERIFICATION_TIMEOUT_MS) {
                    // Verification failed - retry or error
                    if (retryCount < MAX_RETRY_ATTEMPTS) {
                        retryCount++;
                        robot.spindexerServo.setPosition(lastCommandedPosition);
                        rotationState = RotationState.RETRYING;
                        rotationTimer.reset();
                    } else {
                        rotationState = RotationState.ERROR;
                        logError("Position verification failed after " + MAX_RETRY_ATTEMPTS + " retries. " +
                                "Expected: " + expectedDeg + "°, Actual: " + encoderDeg + "°");
                    }
                }
                break;

            case RETRYING:
                // Same as ROTATING - wait for timer then verify again
                double retryTimeout = isWrapRotation ? WRAP_ROTATION_TIME_MS : ROTATION_TIME_MS;
                if (rotationTimer.milliseconds() >= retryTimeout) {
                    rotationState = RotationState.VERIFYING;
                    verificationTimer.reset();
                }
                break;

            case ERROR:
                // Stay in error state until manually cleared
                // Could add auto-recovery logic here if needed
                break;
        }
    }

    /**
     * Clear error state (call from TeleOp if manual recovery is needed).
     */
    public void clearError() {
        if (rotationState == RotationState.ERROR) {
            rotationState = RotationState.IDLE;
            retryCount = 0;
        }
    }

    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return;

        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                currentState = FlickState.Idle;
                break;
        }
    }

    // ==================== LOGGING ====================

    private void logWrapWarning(String direction, double fromPos, double toPos) {
        // This should rarely happen in normal operation
        System.out.println("[SPINDEXER WARNING] Boundary wrap triggered: " +
                direction + " from " + fromPos + " to " + toPos);
    }

    private void logError(String message) {
        System.out.println("[SPINDEXER ERROR] " + message);
    }
}
