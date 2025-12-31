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
    private double angleRange = RobotConstants.Spindexer.ANGLE_RANGE;
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

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        targetPosition = RobotConstants.Spindexer.ENCODER_OFFSET;
        lastTime = System.nanoTime();
        kP = SPINDEXER_PID.p;
        kI = SPINDEXER_PID.i;
        kD = SPINDEXER_PID.d;
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
     * @return true if not rotating, false otherwise
     */
    public boolean isReadyToFlip() {
        return !shouldRotate;
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

    // Runtime PID tuning methods
    public void adjustP(double delta) {
        kP += delta;
        kP = Math.max(0, kP); // Don't go negative
    }

    public void adjustD(double delta) {
        kD += delta;
        kD = Math.max(0, kD); // Don't go negative
    }

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    public void rotationUpdater() {
        if (!shouldRotate) return;
        performPIDRotation();
    }

    private void performPIDRotation() {
        double currentPosition = getServoPosition();
        double error = targetPosition - currentPosition;

        // Take shortest path
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Check if we're close enough (deadband)
        if (Math.abs(error) < angleRange) {
            robot.spindexerServo.setPower(0);
            shouldRotate = false;
            integral = 0;
            return;
        }

        // Calculate dt
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        // Sanity check on dt
        if (dt > 1.0 || dt < 0.001) {
            dt = 0.02; // Default to 50Hz
        }

        // PID calculations with anti-windup
        integral += error * dt;
        integral = Math.max(-50, Math.min(50, integral)); // Clamp integral
        double derivative = (error - lastError) / dt;
        lastError = error;

        double power = (kP * error) + (kI * integral) + (kD * derivative);

        // SAFETY: Check for NaN/Infinite values
        if (!Double.isFinite(power)) {
            robot.spindexerServo.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        // Reduce max power to match turret behavior
        double maxPower = 0.5;

        // Clamp power
        power = Math.max(-maxPower, Math.min(maxPower, power));

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


    @Override
    public void periodic() {
        stateMachinePeriodic();
        rotationUpdater();
    }
}