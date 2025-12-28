package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;
import utility.ShootingStrategy;

import static utility.RobotConstants.Spindexer.FLICK_TIME;
import static utility.RobotConstants.Spindexer.SPINDEXER_PID;

public class Spindexer implements Subsystem {
    private final RobotHardware robot;
    FlickState currentState = FlickState.Idle;
    private double targetPosition;
    private double angleRange = 3;
    private boolean shouldRotate = false;
    private ShootingStrategy.Action[] shootingSequence = null;
    private int sequenceIndex = 0;

    // PID variables - Tuned values from SpindexerPIDFTuningTeleOp
    private double kP;
    private double kI;
    private double kD;
    private double lastError = 0;
    private double integral = 0;
    private long lastTime = 0;

    private ElapsedTime flickerTimer = new ElapsedTime();

    // Stall detection state
    private RobotConstants.Enums.SpindexerRotationState rotationState = RobotConstants.Enums.SpindexerRotationState.IDLE;
    private ElapsedTime stallDetectionTimer = new ElapsedTime();
    private double lastEncoderPosition = 0;
    private int retryAttempts = 0;
    private JamClearanceCallback jamClearanceCallback = null;

    // Interface for jam clearance communication
    public interface JamClearanceCallback {
        void onJamDetected();
        boolean isJamClearingInProgress();
    }

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        targetPosition = RobotConstants.Spindexer.ENCODER_OFFSET;
        lastTime = System.nanoTime();
        kP = SPINDEXER_PID.p;
        kI = SPINDEXER_PID.i;
        kD = SPINDEXER_PID.d;
    }

    public void setJamClearanceCallback(JamClearanceCallback callback) {
        this.jamClearanceCallback = callback;
    }

    private void stateMachinePeriodic() {
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
                robot.spindexerPattern.setBallPatternNone(1); // Clear shooter slot after flick
                currentState = FlickState.Idle; // Return to idle after completion
                break;
        }
    }

    public void triggerFlick() {
        if (currentState == FlickState.Idle) { // Only start if not already running
            currentState = FlickState.Start;
        }
    }

    // ****** STATE QUERIES ******
    public FlickState getCurrentState() {
        return currentState;
    }

    public boolean isIdle() {
        return currentState == FlickState.Idle;
    }

    /**
     * Checks if the spindexer is at rest and ready for flipping
     * @return true if not rotating and rotation state is IDLE, false otherwise
     */
    public boolean isReadyToFlip() {
        return !shouldRotate && rotationState == RobotConstants.Enums.SpindexerRotationState.IDLE;
    }

    public void rotateBy(double positionChange) {
        targetPosition += positionChange;
        // Wrap to [0, 360)
        while (targetPosition >= 360) targetPosition -= 360;
        while (targetPosition < 0) targetPosition += 360;
        shouldRotate = true;

        // Reset PID
        integral = 0;
        lastError = 0;

        // Initialize stall detection
        rotationState = RobotConstants.Enums.SpindexerRotationState.ROTATING;
        stallDetectionTimer.reset();
        lastEncoderPosition = getServoPosition();
        retryAttempts = 0;
    }

    // ****** CATALOGING METHODS ******

    /**
     * Rotates the spindexer to the next slot (120 degrees forward)
     */
    public void rotateToNextSlot() {
        rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
    }

    /**
     * Finds the nearest empty slot from the current intake position (slot 0)
     * Priority order: slot 1 (120° forward), slot 2 (120° backward)
     * @return slot index (1 or 2), or -1 if no empty slots
     */
    public int findNearestEmptySlot() {
        // Check slot 1 first (forward rotation)
        if (robot.spindexerPattern.getBallInSlotX(1) == RobotConstants.Enums.BallColor.None) {
            return 1;
        }
        // Check slot 2 (backward rotation from slot 0)
        if (robot.spindexerPattern.getBallInSlotX(2) == RobotConstants.Enums.BallColor.None) {
            return 2;
        }
        // No empty slots found
        return -1;
    }

    /**
     * Rotates to a specific slot index
     * Assumes current position is at slot 0 (intake position)
     * @param targetSlot Target slot index (0, 1, or 2)
     */
    public void rotateToSlot(int targetSlot) {
        switch (targetSlot) {
            case 0:
                // Already at intake position, no rotation needed
                break;
            case 1:
                // Rotate forward 120° to shooter position
                rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
                break;
            case 2:
                // Rotate backward 120° to top position (shorter path from slot 0)
                rotateBy(RobotConstants.Spindexer.ROTATION_BACKWARD);
                break;
        }
    }

    /**
     * Checks if the spindexer is full (all 3 slots occupied)
     * @return true if all slots contain a ball, false otherwise
     */
    public boolean isFull() {
        return robot.spindexerPattern.getBallInSlotX(0) != RobotConstants.Enums.BallColor.None &&
               robot.spindexerPattern.getBallInSlotX(1) != RobotConstants.Enums.BallColor.None &&
               robot.spindexerPattern.getBallInSlotX(2) != RobotConstants.Enums.BallColor.None;
    }

    /**
     * Catalogs a ball in the specified slot
     * @param slot The slot index (0-2)
     * @param color The color of the ball
     */
    public void catalogBall(int slot, RobotConstants.Enums.BallColor color) {
        robot.spindexerPattern.setBallInSlotX(slot, color);
    }

    /**
     * Gets the color of the ball in the specified slot
     * @param slot The slot index (0-2)
     * @return The ball color
     */
    public RobotConstants.Enums.BallColor getBallInSlot(int slot) {
        return robot.spindexerPattern.getBallInSlotX(slot);
    }

    public double getServoPosition() {
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= 3.3;
        pos *= 360;

        // Wrap to [0, 360)
        while (pos >= 360) pos -= 360;
        while (pos < 0) pos += 360;
        return pos;
    }

    public double getTargetPosition(){
        return targetPosition;
    }

    private boolean detectStall() {
        if (!shouldRotate) return false;

        // Don't check for stalls if we're already near the target
        double currentPosition = getServoPosition();
        double errorToTarget = targetPosition - currentPosition;

        // Normalize error to shortest path
        if (errorToTarget > 180) errorToTarget -= 360;
        if (errorToTarget < -180) errorToTarget += 360;

        // If within acceptable range of target, no stall
        if (Math.abs(errorToTarget) < angleRange) {  // angleRange = 3 degrees
            return false;
        }

        // Existing logic continues...
        double positionChange = Math.abs(currentPosition - lastEncoderPosition);

        // Normalize for wrap-around
        if (positionChange > 180) {
            positionChange = 360 - positionChange;
        }

        // Reset timer if position changed enough
        if (positionChange >= RobotConstants.Spindexer.STALL_POSITION_THRESHOLD) {
            stallDetectionTimer.reset();
            lastEncoderPosition = currentPosition;
            return false;
        }

        // Stalled if stuck for too long
        return stallDetectionTimer.seconds() >= RobotConstants.Spindexer.STALL_DETECTION_TIME;
    }

    public void rotationUpdater() {
        switch (rotationState) {
            case IDLE:
                return;

            case ROTATING:
            case RETRY:
                // Check for stall
                if (detectStall()) {
                    handleStallDetected();
                    return;
                }
                performPIDRotation();
                break;

            case STALLED:
                // Wait for jam clearance
                if (jamClearanceCallback != null && !jamClearanceCallback.isJamClearingInProgress()) {
                    // Clearance complete, retry rotation
                    retryAttempts++;
                    if (retryAttempts >= RobotConstants.Spindexer.MAX_RETRY_ATTEMPTS) {
                        // Give up after max retries
                        robot.spindexerServo.setPower(0);
                        shouldRotate = false;
                        rotationState = RobotConstants.Enums.SpindexerRotationState.IDLE;
                    } else {
                        rotationState = RobotConstants.Enums.SpindexerRotationState.RETRY;
                        stallDetectionTimer.reset();
                        lastEncoderPosition = getServoPosition();
                    }
                }
                break;
        }
    }

    private void handleStallDetected() {
        robot.spindexerServo.setPower(0);
        rotationState = RobotConstants.Enums.SpindexerRotationState.STALLED;
        if (jamClearanceCallback != null) {
            jamClearanceCallback.onJamDetected();
        }
    }

    private void performPIDRotation() {
        double currentPosition = getServoPosition();
        double error = targetPosition - currentPosition;

        // Take shortest path
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Are we close enough?
        if (Math.abs(error) < angleRange) {
            robot.spindexerServo.setPower(0);
            shouldRotate = false;
            rotationState = RobotConstants.Enums.SpindexerRotationState.IDLE;
            integral = 0;
            return;
        }

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        // PID calculations
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp power to [-1, 1]
        power = Math.max(-1, Math.min(1, power));

        robot.spindexerServo.setPower(power);
    }

    public boolean isDoneRotating() {
        double currentPosition = getServoPosition();
        double difference = targetPosition - currentPosition;

        // Take shortest path
        if (difference > 180) difference -= 360;
        if (difference < -180) difference += 360;

        return Math.abs(difference) < angleRange;
    }

    public void startShootingSequence(RobotConstants.MotifPattern goalPattern) {
        shootingSequence = ShootingStrategy.getShootingSequence(
                robot.spindexerPattern,
                goalPattern
        );
        sequenceIndex = 0;
    }

    public ShootingStrategy.Action getNextAction() {
        if (shootingSequence != null && sequenceIndex < shootingSequence.length) {
            return shootingSequence[sequenceIndex];
        }
        return null;
    }

    public void completeCurrentAction() {
        if (shootingSequence != null) {
            sequenceIndex++;
            if (sequenceIndex >= shootingSequence.length) {
                shootingSequence = null;
                sequenceIndex = 0;
            }
        }
    }

    public boolean hasMoreActions() {
        return shootingSequence != null && sequenceIndex < shootingSequence.length;
    }

    public RobotConstants.Enums.SpindexerRotationState getRotationState() {
        return rotationState;
    }

    public int getRetryAttempts() {
        return retryAttempts;
    }

    @Override
    public void periodic() {
        stateMachinePeriodic();
        if(shouldRotate)
            rotationUpdater();
    }
}