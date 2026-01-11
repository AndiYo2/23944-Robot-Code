package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.util.ElapsedTime;
import Constants.EnumConstants;
import Constants.EnumConstants.FlickState;
import Constants.EnumConstants.RotationState;
import Constants.RobotConstants;
import Constants.RobotHardware;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;

import static Constants.SpindexerConstants.*;

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
    private final double angleRange = SpindexerConstants.ANGLE_RANGE;

    // Timers
    private final ElapsedTime flickerTimer = new ElapsedTime();
    private final ElapsedTime settlingTimer = new ElapsedTime();

    // Settling state
    private boolean isSettling = false;

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

    public Spindexer() {
        this.robot = RobotHardware.getInstance();

        // Set safe default position until encoder is ready
        spindPosTracker = 0;
        targetPosition = SPINDEXER_POSITIONS[0];
        // needsInitialization is already true by default
        // Actual position will be read in periodic() when encoder voltage is valid

        // Initialize PID coefficients from constants (can be tuned later)
        cw_kP = SpindexerConstants.SPINDEXER_CW_P;
        cw_kI = SpindexerConstants.SPINDEXER_CW_I;
        cw_kD = SpindexerConstants.SPINDEXER_CW_D;
        cw_kF = SpindexerConstants.SPINDEXER_CW_F;

        ccw_kP = SpindexerConstants.SPINDEXER_CCW_P;
        ccw_kI = SpindexerConstants.SPINDEXER_CCW_I;
        ccw_kD = SpindexerConstants.SPINDEXER_CCW_D;
        ccw_kF = SpindexerConstants.SPINDEXER_CCW_F;

        // Initialize flipper to retracted position
        robot.spindexerFlipperServo.setPosition(SpindexerConstants.FLIPPER_POSITION_RETRACT);
    }

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

    public boolean rotateToColor(EnumConstants.BallColor color) {
        if (rotationState != RotationState.IDLE) return false; // Already rotating

        // Check slot 1 first (already in shooter position)
        if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(1) == color) {
            return true; // Already aligned, no rotation needed
        }

        // Check slot 2 (one CW rotation away)
        if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) == color) {
            rotateCW();
            return true;
        }

        // Check slot 0 (one CCW rotation away)
        if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) == color) {
            rotateCCW();
            return true;
        }
        rotateToNextClosestBall();
        return false;
    }

    public void rotateToNextClosestBall() {
        if(SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(0) != EnumConstants.BallColor.None){
            rotateCCW();
        }
        else if (SpindexerAndMotifStatus.SpindexerPattern.getBallInSlotX(2) != EnumConstants.BallColor.None){
            rotateCW();
        }
    }

    public void rotateToNearestEmptySlot() {
        if (rotationState != RotationState.IDLE) return; // Prevent conflicts
        if (SpindexerAndMotifStatus.SpindexerPattern.isFull()) return;

        double currentPos = getServoPosition();
        double minDistance = Double.MAX_VALUE;
        int nearestSlotIndex = -1;

        // Find nearest empty slot
        for (int i = 0; i < SPINDEXER_POSITIONS.length; i++) {
            if (spindexerPattern.getBallInSlotX(i) == EnumConstants.BallColor.None) {
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

    public void triggerFlick() {
        if (currentState == FlickState.Idle) {
            currentState = FlickState.Start;
        }
    }

    public boolean isRotationIdle() {
        return rotationState == RotationState.IDLE;
    }

    public RotationState getRotationState() {
        return rotationState;
    }

    public boolean isReadyToFlip() {
        return rotationState == RotationState.IDLE && currentState == FlickState.Idle;
    }


    public FlickState getCurrentState() {
        return currentState;
    }


    public int getSpindPosTracker() {
        return spindPosTracker;
    }

    public double getServoPosition() {
        // Convert voltage (0-3.3V) to angle (0-360°)
        double pos = robot.spindexerEncoder.getVoltage();
        pos /= RobotConstants.Encoder.MAX_VOLTAGE;  // Normalize to 0-1
        pos *= RobotConstants.Encoder.FULL_ROTATION_DEGREES;  // Scale to degrees

        // Ensure angle stays in [0, 360) range
        while (pos >= RobotConstants.Encoder.FULL_ROTATION_DEGREES) pos -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        while (pos < 0) pos += RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        return pos;
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public boolean isDoneRotating() {
        double currentPosition = getServoPosition();
        double difference = targetPosition - currentPosition;

        // Take shortest path around circle
        if (difference > RobotConstants.Encoder.ANGLE_UPPER_BOUND) difference -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
        if (difference < RobotConstants.Encoder.ANGLE_LOWER_BOUND) difference += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

        return Math.abs(difference) < angleRange;
    }



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


    @Deprecated
    public void setSpindexerPIDF(double kP, double kI, double kD, double kF) {
        setCWPIDF(kP, kI, kD, kF);
        setCCWPIDF(kP, kI, kD, kF);
    }

    private boolean isEncoderReady() {
        double currentVoltage = robot.spindexerEncoder.getVoltage();

        // First call - record initial voltage
        if (initialEncoderVoltage == -1) {
            initialEncoderVoltage = currentVoltage;
            return false; // Not ready yet, just recorded baseline
        }

        // Check if voltage has changed from initial reading
        // Encoder is ready when it gives a different value than the initial stuck reading
        return Math.abs(currentVoltage - initialEncoderVoltage) > SpindexerConstants.ENCODER_READY_THRESHOLD;
    }

    private int getNearestStartIndex() {
        double currentPos = getServoPosition();
        if (isWithinRange(currentPos, SPINDEXER_POSITIONS[0])) return 0;
        if (isWithinRange(currentPos, SPINDEXER_POSITIONS[1])) return 1;
        return 2;
    }


    private boolean isWithinRange(double angle, double target) {
        double diff = Math.abs(angle - target);
        if (diff > RobotConstants.Encoder.ANGLE_UPPER_BOUND) {
            diff = RobotConstants.Encoder.FULL_ROTATION_DEGREES - diff;
        }
        return diff <= SpindexerConstants.ANGLE_WITHIN_RANGE_THRESHOLD;
    }

    private double calculateAngularDistance(double angle1, double angle2) {
        double diff = Math.abs(angle1 - angle2);
        // Take the shorter path around the circle
        return Math.min(diff, RobotConstants.Encoder.FULL_ROTATION_DEGREES - diff);
    }

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

    private void rotationUpdater() {
        // Always run PID controller to actively maintain position
        performPIDRotation();
    }

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
                if (settlingTimer.seconds() >= RobotConstants.ShootingSequence.SPINDEXER_SETTLING_TIME) {
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
            while (error > RobotConstants.Encoder.ANGLE_UPPER_BOUND) error -= RobotConstants.Encoder.FULL_ROTATION_DEGREES;
            while (error < RobotConstants.Encoder.ANGLE_LOWER_BOUND) error += RobotConstants.Encoder.FULL_ROTATION_DEGREES;

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

    private void flipperStateMachinePeriodic() {
        if (currentState == FlickState.Idle) return; // Don't run unless activated

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
                currentState = FlickState.Idle; // Return to idle after completion
                break;
        }
    }
}
