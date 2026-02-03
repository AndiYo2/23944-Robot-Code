package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import utility.RobotHardware;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

import static Constants.RobotConstants.Robot.MIN_SERVO_SAFE_POSITION;
import static Constants.SpindexerConstants.*;

/**
 * Spindexer subsystem - Barebones degree-based position control.
 *
 * Uses 6 servo positions (0°, 60°, 120°, 180°, 240°, 300°) mapped to 3 physical slots.
 * Each slot has two equivalent positions due to 2:1 gear ratio:
 *   - Slot 0: 0°, 180°
 *   - Slot 1: 60°, 240°
 *   - Slot 2: 120°, 300°
 *
 * Boundary wrapping:
 *   - At 300°, CW wraps to 180°
 *   - At 0°, CCW wraps to 120°
 */
public class Spindexer extends SubsystemBase {

    private final RobotHardware robot;

    private FlickState currentState = FlickState.Idle;
    private final ElapsedTime flickerTimer = new ElapsedTime();

    private final ElapsedTime rotationCooldown = new ElapsedTime();

    private int currentDegrees = EMPTY_RESET_DEGREES;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();

        robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_RETRACT);

        currentDegrees = EMPTY_RESET_DEGREES;
        robot.spindexerServo.setPosition(degreesToServoPosition(EMPTY_RESET_DEGREES));
    }

    /**
     * Convert degrees to servo position (0-1 range).
     * Axon servo has 355° range, so position = degrees / 355.
     * Applies SPINDEXER_OFFSET to all rotations.
     * Avoids exact 0.0 which can cause servo issues.
     */
    private double degreesToServoPosition(int degrees) {
        double position = (degrees + SpindexerConstants.SPINDEXER_OFFSET) / SERVO_DEGREES_PER_UNIT;
        if (position < MIN_SERVO_SAFE_POSITION) position = MIN_SERVO_SAFE_POSITION;
        return position;
    }

    // ==================== ROTATION METHODS ====================

    public void rotateCW() {
        int newDegrees = currentDegrees + DEGREE_INCREMENT;

        // Boundary wrapping
        if (newDegrees > MAX_SERVO_DEGREES) {
            newDegrees = CW_WRAP_TO_DEG;  // 180°
        }

        currentDegrees = newDegrees;
        rotationCooldown.reset();
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCW();
    }

    public void rotateCCW() {
        int newDegrees = currentDegrees - DEGREE_INCREMENT;

        if (newDegrees < 0) {
            newDegrees = CCW_WRAP_TO_DEG;  // 120°
        }

        currentDegrees = newDegrees;
        rotationCooldown.reset();
        SpindexerAndMotifStatus.SpindexerPattern.rotateBallsCCW();
    }


    /**
     * Smart direction choice to minimize wrapping.
     */
    private void rotateAwayFromBoundary() {
        // Near 0° boundary (within one increment)
        if (currentDegrees <= DEGREE_INCREMENT) {
            rotateCW();   // Go toward higher positions
        }
        // Near max boundary (within one increment)
        else if (currentDegrees >= MAX_SERVO_DEGREES - DEGREE_INCREMENT) {
            rotateCCW();  // Go toward lower positions
        } else {
            // Safe zone - prefer toward 120° (slot 2) for next load cycle
            if (currentDegrees > CCW_WRAP_TO_DEG) {
                rotateCCW();
            } else {
                rotateCW();
            }
        }
    }


    public void rotateToNextBall(){
        if(SpindexerConstants.currentMode == EnumConstants.ShootingMode.Fast){
            rotateCCW();
        }else{
            if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) != EnumConstants.BallColor.None){
                rotateCCW();
            }else if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) != EnumConstants.BallColor.None){
                rotateCW();
            }
        }
    }


    /**
     * Reset to empty position (0°) and clear ball tracking.
     */
    public void resetToEmptyPosition() {
        currentDegrees = EMPTY_RESET_DEGREES;
        SpindexerAndMotifStatus.SpindexerPattern.clearAll();
    }


    // ==================== SERVO CONTROL ====================

    /**
     * Set spindexer to a specific degree position.
     * @param degrees Target position in servo degrees (0-300)
     */
    public void setDegree(int degrees) {
        if (degrees < 0 || degrees > MAX_SERVO_DEGREES) return;

        currentDegrees = degrees;
    }

    // ==================== FLIPPER METHODS ====================

    public void triggerFlick() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
            // Automatically remove ball from slot 1 (shooter slot) when flicking
            // Safe even if slot is empty (sets None to None)
            SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
        }
    }

    // ==================== STATE QUERIES ====================

    public boolean isRotationIdle() {
        return rotationCooldown.seconds() > ROTATION_TIME;
    }

    public boolean isReadyToFlip() {
        return currentState == FlickState.Idle;
    }

    public FlickState getCurrentState() {
        return currentState;
    }

    /**
     * Get current position in degrees.
     */
    public int getCurrentDegrees() {
        return currentDegrees;
    }

    /**
     * Get current servo position (for debugging).
     */
    public double getServoPosition() {
        return degreesToServoPosition(currentDegrees);
    }

    /**
     * Get current slot index (0, 1, or 2) based on degree position.
     */
    public int getSpindPosTracker() {
        return (currentDegrees / DEGREE_INCREMENT) % SLOTS_COUNT;
    }

    /**
     * Get target position in degrees (for telemetry compatibility).
     */
    public int getTargetPosition() {
        return currentDegrees;
    }

    /**
     * Check if rotation is complete.
     */
    public boolean isDoneRotating() {
        return rotationCooldown.seconds() > ROTATION_TIME;
    }

    // ==================== PERIODIC UPDATES ====================

    @Override
    public void periodic() {
        flipperStateMachinePeriodic();
        robot.spindexerServo.setPosition(degreesToServoPosition(currentDegrees));
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
                if(flickerTimer.seconds() < .3) break;
                currentState = FlickState.Idle;
                break;
        }
    }
}
