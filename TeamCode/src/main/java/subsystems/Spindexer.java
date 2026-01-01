package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotConstants.Enums.RotationState;
import utility.RobotHardware;
import utility.SpindexerAndMotifStatus;

import static utility.RobotConstants.Spindexer.*;

/**
 * Spindexer Subsystem - Manages a 3-slot ball indexer with rotation and flicking mechanisms.
 *
 * <p>This subsystem controls:
 * <ul>
 *   <li>Rotation servo - Rotates indexer to position balls at shooter entrance</li>
 *   <li>Flipper servo - Flicks balls from indexer into shooter</li>
 *   <li>Ball tracking - Monitors which slots contain balls and their colors</li>
 * </ul>
 *
 * <p>The spindexer has 3 slots positioned at 62°, 182°, and 302° (120° apart).
 * Rotation uses PID control for smooth, accurate positioning.
 *
 * <p><b>State Machines:</b>
 * <ul>
 *   <li>Rotation: IDLE (ready) ↔ ROTATING (moving)</li>
 *   <li>Flipper: Idle → Start → Extended → Retracted → Idle</li>
 * </ul>
 *
 * @see RobotConstants.Spindexer for hardware mappings and tuning constants
 * @see utility.RobotConstants.Enums.RotationState
 * @see utility.RobotConstants.Enums.FlickState
 */
public class Spindexer implements Subsystem {

    // Hardware reference
    private final RobotHardware robot;

    // State tracking
    private RotationState rotationState = RotationState.IDLE;
    private FlickState currentState = FlickState.Idle;

    // Position tracking
    private int targetPosition;
    private int spindPosTracker;

    // PID state
    private double lastError = 0;
    private double integral = 0;

    // Configuration
    private final double angleRange = RobotConstants.Spindexer.ANGLE_RANGE;

    // Timers
    private final ElapsedTime flickerTimer = new ElapsedTime();

    // ====================================================================
    // CONSTRUCTOR
    // ====================================================================

    /**
     * Initializes the Spindexer subsystem.
     * Automatically finds and moves to the nearest slot position on startup.
     */
    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        spindPosTracker = getNearestStartIndex();
        targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
    }

    // ====================================================================
    // ROTATION CONTROL METHODS
    // ====================================================================

    /**
     * Rotates the spindexer clockwise to the next slot position.
     * Automatically prevents rotation if already rotating (state protection).
     *
     * <p>The spindexer has 3 slots, so calling this 3 times returns to the start position.
     */
    public void rotateCW() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts

        spindPosTracker = (spindPosTracker + 1) % SPINDEXER_POSITIONS.length;
        targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
        rotationState = RotationState.ROTATING;
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCW();
    }

    /**
     * Rotates the spindexer counter-clockwise to the previous slot position.
     * Automatically prevents rotation if already rotating (state protection).
     *
     * <p>The spindexer has 3 slots, so calling this 3 times returns to the start position.
     */
    public void rotateCCW() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts

        spindPosTracker = (spindPosTracker - 1 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
        targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
        rotationState = RotationState.ROTATING;
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCCW();
    }

    /**
     * Rotates the spindexer clockwise to the next slot position.
     * This is an alias for {@link #rotateCW()}.
     */
    public void rotateToNextSlot() {
        rotateCW();
    }

    /**
     * Rotates to the nearest slot that doesn't contain a ball.
     * Does nothing if the spindexer is full or already rotating.
     *
     * <p>With only 3 slots, the nearest empty slot is always at most 1 rotation away.
     * Uses rotateCW() or rotateCCW() to properly update ball tracking.
     */
    public void rotateToNearestEmptySlot() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts
        if (SpindexerAndMotifStatus.SpindexerPattern.isFull()) return;

        double currentPos = getServoPosition();
        double minDistance = Double.MAX_VALUE;
        int nearestSlotIndex = -1;

        // Find nearest empty slot
        for (int i = 0; i < SPINDEXER_POSITIONS.length; i++) {
            if (spindexerPattern.getBallInSlotX(i) == RobotConstants.Enums.BallColor.None) {
                double distance = calculateAngularDistance(currentPos, SPINDEXER_POSITIONS[i]);
                if (distance < minDistance) {
                    minDistance = distance;
                    nearestSlotIndex = i;
                }
            }
        }

        if (nearestSlotIndex == -1) return; // No empty slots found
        if (nearestSlotIndex == spindPosTracker) return; // Already at empty slot

        // Calculate which direction is shorter
        int stepsToTarget = nearestSlotIndex - spindPosTracker;
        int stepsCW = (stepsToTarget + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
        int stepsCCW = (SPINDEXER_POSITIONS.length - stepsCW) % SPINDEXER_POSITIONS.length;

        // Rotate once in the shorter direction
        if (stepsCW <= stepsCCW) {
            rotateCW();
        } else {
            rotateCCW();
        }
    }

    // ====================================================================
    // FLIPPER CONTROL METHODS
    // ====================================================================

    /**
     * Triggers the flipper to flick a ball from the spindexer into the shooter.
     * Only starts if the flipper is currently idle (prevents overlapping flicks).
     *
     * <p>The flick sequence runs automatically through the state machine:
     * Idle → Start → Extended → Retracted → Idle
     */
    public void triggerFlick() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }


    // ====================================================================
    // STATE QUERY METHODS
    // ====================================================================

    /**
     * Checks if rotation is idle and ready for new commands.
     *
     * @return true if idle, false if rotating
     */
    public boolean isRotationIdle() {
        return rotationState == RotationState.IDLE;
    }

    /**
     * Checks if the spindexer is currently rotating.
     *
     * @return true if rotating, false if idle
     */
    public boolean isRotating() {
        return rotationState == RotationState.ROTATING;
    }

    /**
     * Gets the current rotation state.
     *
     * @return Current rotation state (IDLE or ROTATING)
     */
    public RotationState getRotationState() {
        return rotationState;
    }

    /**
     * Checks if the spindexer is at rest and ready for flipping.
     *
     * @return true if both rotation and flipper are idle (safe to start flick sequence)
     */
    public boolean isReadyToFlip() {
        return rotationState == RotationState.IDLE && currentState == FlickState.Idle;
    }

    /**
     * Gets the current flipper state.
     *
     * @return Current flipper state (Idle, Start, Extended, or Retracted)
     */
    public FlickState getCurrentState() {
        return currentState;
    }

    /**
     * Checks if the flipper is idle.
     *
     * @return true if flipper state is Idle
     */
    public boolean isIdle() {
        return currentState == FlickState.Idle;
    }

    /**
     * Gets the current servo position in degrees.
     *
     * <p>Reads the encoder voltage and converts it to an angle (0-360°).
     *
     * @return Current servo position in degrees (0-360)
     */
    public double getServoPosition() {
        // Convert voltage (0-3.3V) to angle (0-360°)
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= 3.3;  // Normalize to 0-1
        pos *= 360;  // Scale to degrees

        // Ensure angle stays in [0, 360) range
        while (pos >= 360) pos -= 360;
        while (pos < 0) pos += 360;
        return pos;
    }

    /**
     * Gets the target position the spindexer is rotating to.
     *
     * @return Target position in degrees
     */
    public int getTargetPosition() {
        return targetPosition;
    }

    /**
     * Checks if rotation is complete (within acceptable error range).
     *
     * @return true if current position is within tolerance of target position
     */
    public boolean isDoneRotating() {
        double currentPosition = getServoPosition();
        double difference = targetPosition - currentPosition;

        // Take shortest path around circle
        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        return Math.abs(difference) < angleRange;
    }

    // ====================================================================
    // INTERNAL HELPERS
    // ====================================================================

    /**
     * Finds which of the 3 slot positions is closest to the current servo position.
     * Used during initialization to snap to the nearest valid slot.
     */
    private int getNearestStartIndex() {
        double currentPos = getServoPosition();

        int nearestIndex = 0;
        double minDistance = calculateAngularDistance(currentPos, SPINDEXER_POSITIONS[0]);

        for (int i = 1; i < SPINDEXER_POSITIONS.length; i++) {
            double distance = calculateAngularDistance(currentPos, SPINDEXER_POSITIONS[i]);
            if (distance < minDistance) {
                minDistance = distance;
                nearestIndex = i;
            }
        }

        return nearestIndex;
    }

    /**
     * Calculates the shortest angular distance between two angles.
     * Accounts for wraparound at 360°/0°.
     *
     * @param angle1 First angle in degrees
     * @param angle2 Second angle in degrees
     * @return Shortest angular distance in degrees
     */
    private double calculateAngularDistance(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2);
        // Take the shorter path around the circle
        return Math.min(diff, 360 - diff);
    }

    // ====================================================================
    // PERIODIC UPDATES
    // ====================================================================

    /**
     * Main periodic update method called by the subsystem scheduler.
     * Updates both flipper and rotation state machines.
     */
    @Override
    public void periodic() {
        flipperStateMachinePeriodic();
        rotationUpdater();
    }

    /**
     * Called by periodic() - runs PID controller if rotating.
     */
    private void rotationUpdater() {
        if (rotationState == RotationState.IDLE) return;
        performPIDRotation();
    }

    /**
     * Internal PID controller for smooth rotation to target position.
     *
     * <p>Uses proportional-integral-derivative control with:
     * <ul>
     *   <li>Shortest path calculation (handles 360° wraparound)</li>
     *   <li>Anti-windup integral clamping</li>
     *   <li>Fixed dt (assumes ~50Hz loop rate)</li>
     *   <li>Power clamping to ±0.5 for safety</li>
     * </ul>
     *
     * <p>Automatically returns to IDLE state when within acceptable error range.
     */
    private void performPIDRotation() {
        double currentPosition = getServoPosition();
        double error = targetPosition - currentPosition;

        // Take shortest path around circle (e.g., -350° error becomes +10°)
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Check if we're close enough (deadband)
        if (Math.abs(error) < angleRange) {
            robot.spindexerServo.setPower(0);
            rotationState = RotationState.IDLE;
            integral = 0;
            return;
        }

        // Fixed dt assuming ~50Hz loop rate
        double dt = 0.02;

        // PID calculations with anti-windup
        integral += error * dt;
        integral = Math.max(-50, Math.min(50, integral)); // Anti-windup: prevent integral from growing unbounded
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (SPINDEXER_PID.p * error) +
                       (SPINDEXER_PID.i * integral) +
                       (SPINDEXER_PID.d * derivative);

        // Clamp power to safe limits
        power = Math.max(-0.5, Math.min(0.5, power));

        robot.spindexerServo.setPower(power);
    }

    /**
     * State machine for flipper servo actuation sequence.
     * Runs the flick sequence: Idle → Start → Extended → Retracted → Idle
     */
    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return; // Don't run unless activated

        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.spindexerFlipperServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < FLICK_TIME) break;
                robot.spindexerFlipperServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                spindexerPattern.setBallPatternNone(1); // Clear shooter slot after flick
                currentState = FlickState.Idle; // Return to idle after completion
                break;
        }
    }
}
