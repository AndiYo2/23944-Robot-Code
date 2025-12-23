package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import utility.RobotConstants;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotHardware;
import utility.ShootingStrategy;

import static utility.RobotConstants.Spindexer.FLICK_TIME;

public class Spindexer implements Subsystem {
    private final RobotHardware robot;
    FlickState currentState = FlickState.Idle;
    private double targetPosition;
    private double angleRange = 3;
    private boolean shouldRotate = false;
    private ShootingStrategy.Action[] shootingSequence = null;
    private int sequenceIndex = 0;

    // PID variables - Tuned values from SpindexerPIDFTuningTeleOp
    private double kP = 0.0122;
    private double kI = 0.0;
    private double kD = 0.0005;
    private double lastError = 0;
    private double integral = 0;
    private long lastTime = 0;

    private ElapsedTime flickerTimer = new ElapsedTime();

    public Spindexer() {
        this.robot = RobotHardware.getInstance();
        targetPosition = 72;
        lastTime = System.nanoTime();
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

    public void rotationUpdater() {
        double currentPosition = getServoPosition();
        double error = targetPosition - currentPosition;

        // Take shortest path
        if (error > 180) error -= 360;
        if (error < -180) error += 360;

        // Are we close enough?
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

    @Override
    public void periodic() {
        stateMachinePeriodic();
        if(shouldRotate)
            rotationUpdater();
    }
}