package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import utility.RobotHardware;
import Constants.ShootingSequenceConstants;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

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

    // Servo dirty flag — only write when position actually changes
    private double lastSpindexerServoPosition = -1.0;
    private static final double SERVO_EPSILON = 0.001;

    public Spindexer() {
        this.robot = RobotHardware.getInstance();

        currentDegrees = EMPTY_RESET_DEGREES;
    }

    public void initServoPositions() {
        robot.spindexerFlipperServo.setPosition(ShootingSequenceConstants.SPINDEXER_FLIPPER_RETRACT);
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
        if (position < 0.01) position = 0.01;
        return position;
    }

    // ==================== ROTATION METHODS ====================

    public void rotateCW() {
        int newDegrees = currentDegrees + DEGREE_INCREMENT;

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

    // ==================== FLIPPER METHODS ====================

    public void triggerFlick() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
            // Automatically remove ball from slot 1 (shooter slot) when flicking
            // Safe even if slot is empty (sets None to None)
            SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
        }
    }

    public void extendFlipper() {
        robot.spindexerFlipperServo.setPosition(ShootingSequenceConstants.SPINDEXER_FLIPPER_EXTENDED);
        SpindexerAndMotifStatus.SpindexerPattern.setBallInSlotX(1, EnumConstants.BallColor.None);
    }

    public void retractFlipper() {
        robot.spindexerFlipperServo.setPosition(ShootingSequenceConstants.SPINDEXER_FLIPPER_RETRACT);
    }

    // ==================== STATE QUERIES ====================

    public boolean isRotationIdle() {
        return rotationCooldown.seconds() > ShootingSequenceConstants.SPINDEXER_ROTATION_TIME;
    }

    public boolean isAtTargetPosition() {
        double actual = robot.spindexerEncoder.getVoltage() / 3.3;
        double target = degreesToServoPosition(currentDegrees);
        return Math.abs(actual - target) < ShootingSequenceConstants.SPINDEXER_POSITION_TOLERANCE;
    }

    public boolean isReadyToFlip() {
        return currentState == FlickState.Idle;
    }

    public FlickState getCurrentState() {
        return currentState;
    }

    public int getCurrentDegrees() {
        return currentDegrees;
    }

    public double getServoPosition() {
        return degreesToServoPosition(currentDegrees);
    }

    public int getTargetPosition() {
        return currentDegrees;
    }


    // ==================== PERIODIC UPDATES ====================

    @Override
    public void periodic() {
        flipperStateMachinePeriodic();
        double targetPos = degreesToServoPosition(currentDegrees);
        if (Math.abs(targetPos - lastSpindexerServoPosition) > SERVO_EPSILON) {
            robot.spindexerServo.setPosition(targetPos);
            lastSpindexerServoPosition = targetPos;
        }
    }

    private void flipperStateMachinePeriodic() {
        switch (currentState) {
            case Idle:
                break;
            case Start:
                robot.spindexerFlipperServo.setPosition(ShootingSequenceConstants.SPINDEXER_FLIPPER_EXTENDED);
                flickerTimer.reset();
                currentState = FlickState.Extended;
                break;
            case Extended:
                if (flickerTimer.seconds() < ShootingSequenceConstants.SPINDEXER_FLICK_TIME) break;
                robot.spindexerFlipperServo.setPosition(ShootingSequenceConstants.SPINDEXER_FLIPPER_RETRACT);
                flickerTimer.reset();
                currentState = FlickState.Retracted;
                break;
            case Retracted:
                if(flickerTimer.seconds() < ShootingSequenceConstants.SPINDEXER_RETRACT_DELAY) break;
                currentState = FlickState.Idle;
                break;
        }
    }
}
