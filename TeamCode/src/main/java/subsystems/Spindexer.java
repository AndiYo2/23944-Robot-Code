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

    // Tunable PID coefficients (can be updated via setSpindexerPIDF)
    private double cw_kP, cw_kI, cw_kD, cw_kF;
    private double ccw_kP, ccw_kI, ccw_kD, ccw_kF;

    // Configuration
    private final double angleRange = RobotConstants.Spindexer.ANGLE_RANGE;

    // Timers
    private final ElapsedTime flickerTimer = new ElapsedTime();
    private final ElapsedTime settlingTimer = new ElapsedTime();

    // Settling state
    private boolean isSettling = false;
    private static final double SETTLING_TIME = 0.1; // Require 100ms stable before stopping

    // Debug tracking
    private double initPosition = 0;
    private int initIndex = 0;

    // Debug mode - bypass PID for raw servo testing
    private boolean debugBypassPID = false;
    private double debugManualPower = 0.0;
    private double lastPIDOutput = 0.0;

    // Initialization state
    private boolean needsInitialization = true;
    private double initialEncoderVoltage = -1; // Track first voltage reading

    // ====================================================================
    // CONSTRUCTOR
    // ====================================================================

    /**
     * Initializes the Spindexer subsystem.
     * Sets safe default position. Actual position will be determined in periodic()
     * once encoder is ready (avoids 0V reading during init).
     */
    public Spindexer() {
        this.robot = RobotHardware.getInstance();

        // Set safe default position until encoder is ready
        spindPosTracker = 0;
        targetPosition = SPINDEXER_POSITIONS[0];
        // needsInitialization is already true by default
        // Actual position will be read in periodic() when encoder voltage is valid

        // Initialize PID coefficients from constants (can be tuned later)
        cw_kP = RobotConstants.Spindexer.SPINDEXER_CW_P;
        cw_kI = RobotConstants.Spindexer.SPINDEXER_CW_I;
        cw_kD = RobotConstants.Spindexer.SPINDEXER_CW_D;
        cw_kF = RobotConstants.Spindexer.SPINDEXER_CW_F;

        ccw_kP = RobotConstants.Spindexer.SPINDEXER_CCW_P;
        ccw_kI = RobotConstants.Spindexer.SPINDEXER_CCW_I;
        ccw_kD = RobotConstants.Spindexer.SPINDEXER_CCW_D;
        ccw_kF = RobotConstants.Spindexer.SPINDEXER_CCW_F;

        // Initialize flipper to retracted position
        robot.spindexerFlipperServo.setPosition(RobotConstants.Spindexer.FLIPPER_POSITION_RETRACT);
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
    public void rotateCCW() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts

        robot.spindexerPID.reset();
        // Set CCW-specific PID gains (against gravity) - uses tunable values
        robot.spindexerPID.setPIDF(ccw_kP, ccw_kI, ccw_kD, ccw_kF);

        spindPosTracker = (spindPosTracker + 1) % SPINDEXER_POSITIONS.length;
        targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
        rotationState = RotationState.ROTATING;
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCCW();
    }

    /**
     * Rotates the spindexer counter-clockwise to the previous slot position.
     * Automatically prevents rotation if already rotating (state protection).
     *
     * <p>The spindexer has 3 slots, so calling this 3 times returns to the start position.
     */
    public void rotateCW() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts

        robot.spindexerPID.reset();
       // Set CW-specific PID gains (with gravity assist) - uses tunable values
        robot.spindexerPID.setPIDF(cw_kP, cw_kI, cw_kD, cw_kF);

        spindPosTracker = (spindPosTracker - 1 + SPINDEXER_POSITIONS.length) % SPINDEXER_POSITIONS.length;
        targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
        rotationState = RotationState.ROTATING;
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCW();
    }

    public boolean rotateToColor(RobotConstants.Enums.BallColor color){
        if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1) == color){
            return true;
        }
        else if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) == color){
            rotateCW();
            return true;
        } else if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) == color){
            rotateCCW();
            return true;
        }
        rotateToNextClosestBall();
        return false;
        }

    /**
     * Rotates the spindexer clockwise to the next slot position.
     * This is an alias for {@link #rotateCCW()}.
     */
    public void rotateToNextClosestBall() {
        if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) != RobotConstants.Enums.BallColor.None){
            rotateCCW();
        }
        else if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) != RobotConstants.Enums.BallColor.None){
            rotateCW();
        }
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
     * Gets the current slot index (0, 1, or 2).
     *
     * @return Current slot index tracker
     */
    public int getSpindPosTracker() {
        return spindPosTracker;
    }

    /**
     * Gets the encoder position that was read during initialization.
     *
     * @return Initial position in degrees (0-360)
     */
    public double getInitPosition() {
        return initPosition;
    }

    /**
     * Gets the index that was selected during initialization.
     *
     * @return Initial slot index (0, 1, or 2)
     */
    public int getInitIndex() {
        return initIndex;
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

/**
     * Updates the CW PIDF coefficients for spindexer rotation control.
     * Used for live tuning during TeleOp.
     * Also immediately applies to the active PID controller for real-time tuning.
     *
     * @param kP Proportional coefficient
     * @param kI Integral coefficient
     * @param kD Derivative coefficient
     * @param kF Feedforward coefficient
     */
    public void setCWPIDF(double kP, double kI, double kD, double kF) {
        cw_kP = kP;
        cw_kI = kI;
        cw_kD = kD;
        cw_kF = kF;
        // Immediately apply to active PID controller for real-time tuning
        robot.spindexerPID.setP(kP);
        robot.spindexerPID.setI(kI);
        robot.spindexerPID.setD(kD);
        robot.spindexerPID.setF(kF);
    }

    /**
     * Updates the CCW PIDF coefficients for spindexer rotation control.
     * Used for live tuning during TeleOp.
     * Also immediately applies to the active PID controller for real-time tuning.
     *
     * @param kP Proportional coefficient
     * @param kI Integral coefficient
     * @param kD Derivative coefficient
     * @param kF Feedforward coefficient
     */
    public void setCCWPIDF(double kP, double kI, double kD, double kF) {
        ccw_kP = kP;
        ccw_kI = kI;
        ccw_kD = kD;
        ccw_kF = kF;
        // Immediately apply to active PID controller for real-time tuning
        robot.spindexerPID.setP(kP);
        robot.spindexerPID.setI(kI);
        robot.spindexerPID.setD(kD);
        robot.spindexerPID.setF(kF);
    }

    /**
     * Gets the actual PIDF coefficients currently in the PID controller.
     * Useful for debugging to verify values were actually set.
     * @return array of [P, I, D, F] values
     */
    public double[] getActivePIDFCoefficients() {
        return robot.spindexerPID.getCoefficients();
    }

    /**
     * Gets the last PID output value (for debugging).
     * @return the correction value sent to servo
     */
    public double getLastPIDOutput() {
        return lastPIDOutput;
    }

    /**
     * Gets the last error value (for debugging).
     * @return error in degrees
     */
    public double getLastError() {
        return lastError;
    }

    /**
     * Updates the PIDF coefficients for spindexer rotation control.
     * Legacy method - updates both CW and CCW to the same values.
     * Used for live tuning during TeleOp.
     *
     * @param kP Proportional coefficient
     * @param kI Integral coefficient
     * @param kD Derivative coefficient
     * @param kF Feedforward coefficient
     * @deprecated Use setCWPIDF or setCCWPIDF instead for direction-specific tuning
     */
    @Deprecated
    public void setSpindexerPIDF(double kP, double kI, double kD, double kF) {
        setCWPIDF(kP, kI, kD, kF);
        setCCWPIDF(kP, kI, kD, kF);
    }

    // ====================================================================
    // INTERNAL HELPERS
    // ====================================================================

    /**
     * Checks if the encoder has initialized and is providing valid voltage readings.
     * Detects when voltage changes from the initial reading, indicating encoder is ready.
     *
     * @return true if encoder voltage has changed from initial reading
     */
    private boolean isEncoderReady() {
        double currentVoltage = robot.spindexerEncoder.getVoltage();

        // First call - record initial voltage
        if (initialEncoderVoltage == -1) {
            initialEncoderVoltage = currentVoltage;
            return false; // Not ready yet, just recorded baseline
        }

        // Check if voltage has changed from initial reading
        // Encoder is ready when it gives a different value than the initial stuck reading
        return Math.abs(currentVoltage - initialEncoderVoltage) > 0.01;
    }

    /**
     * Finds which of the 3 slot positions is closest to the current servo position.
     * Used during initialization to snap to the nearest valid slot.
     */
    private int getNearestStartIndex() {
        double currentPos = getServoPosition();
        if (isWithinRange(currentPos, SPINDEXER_POSITIONS[0])) return 0;
        if (isWithinRange(currentPos, SPINDEXER_POSITIONS[1])) return 1;
        return 2;
    }

    /**
     * Checks if an angle is within ±60 degrees of a target angle.
     * Handles wraparound between 0° and 359°.
     *
     * @param angle  Angle to check (0-359)
     * @param target Target angle (0-359)
     * @return true if angle is within ±60° of target
     */
    private boolean isWithinRange(double angle, double target) {
        double diff = Math.abs(angle - target);
        if (diff > 180) {
            diff = 360 - diff;
        }
        return diff <= 60;
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
        // Lazy initialization - wait for encoder to be ready (avoids 0V reading at init)
        if (needsInitialization && isEncoderReady()) {
            initPosition = getServoPosition();
            initIndex = getNearestStartIndex();
            spindPosTracker = initIndex;
            targetPosition = SPINDEXER_POSITIONS[spindPosTracker];
            needsInitialization = false;
        }

        flipperStateMachinePeriodic();
        rotationUpdater();

    }

    /**
     * Called by periodic() - runs PID controller continuously to maintain position.
     * Always runs regardless of state to provide active position holding.
     */
    private void rotationUpdater() {
        // Always run PID controller to actively maintain position
        performPIDRotation();
    }

    /**
     * Internal PID controller for smooth rotation and active position holding.
     *
     * <p>Features:
     * <ul>
     *   <li>Shortest path calculation (handles 360° wraparound)</li>
     *   <li>Continuous operation - NEVER stops correcting position</li>
     *   <li>Active position holding - resists external forces and drift</li>
     *   <li>Settling time verification before declaring rotation complete</li>
     * </ul>
     *
     * <p>The controller runs continuously. State transitions to IDLE after settling,
     * but corrections continue to maintain exact position.
     */
    private void performPIDRotation() {
        double currentPosition = getServoPosition();
        boolean withinTolerance = isDoneRotating();

        // Update state based on settling (for external code to know when rotation is "done")
        if (rotationState == RotationState.ROTATING) {
            if (withinTolerance) {
                // Start settling timer if we just entered tolerance zone
                if (!isSettling) {
                    isSettling = true;
                    settlingTimer.reset();
                }

                // Check if we've been stable long enough to declare rotation complete
                if (settlingTimer.seconds() >= SETTLING_TIME) {
                    rotationState = RotationState.IDLE;
                    isSettling = false;
                }
            } else {
                // Outside tolerance - reset settling
                isSettling = false;
            }
        }

        // ALWAYS calculate and apply PIDF correction (active position holding)
        double correction = 0.0;
        double error = 0.0;

        if (!Double.isNaN(currentPosition)) {
            // Calculate the shortest angular path (handle wraparound)
            error = targetPosition - currentPosition;

            // Normalize error to [-180, 180] range for shortest path
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            // Create a "virtual" target that's on the shortest path from current position
            double wrappedTarget = currentPosition + error;

            // Calculate PIDF correction (feedforward is handled internally by PIDFController)
            correction = robot.spindexerPID.calculate(currentPosition, wrappedTarget);
        }

        // Store for debug telemetry
        lastPIDOutput = correction;
        lastError = error;

        // Always apply correction - never let the servo coast
        robot.spindexerServo.setPower(correction);
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
