package utility;

import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.CatalogingCases;
import Constants.EnumConstants.ShootingMode;
import Constants.RobotConstants;
import Constants.SpindexerConstants;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Intake;
import subsystems.Spindexer;

import static utility.SpindexerAndMotifStatus.SpindexerPattern;

/**
 * CatalogManager - Pre-emptive 3-ball catalogging system
 *
 * Supports two modes:
 * - Sorted mode: Fills all 3 spindexer slots (300 -> 240 -> 180)
 * - Fast mode: Loads 1st ball directly into shooter, remaining into spindexer
 *
 * Fast mode sequence:
 *   1. Position to 240 (shooter slot at intake)
 *   2. Flip spindexer + run intake (ball goes directly to shooter)
 *   3. Rotate to 180
 *   4. Intake remaining balls
 *   5. Ready to shoot immediately
 */
public class CatalogManager {
    // Subsystems
    private final Intake intake;
    private final Spindexer spindexer;
    private final Telemetry telemetry;
    private ShootingSequenceManager sequenceManager;

    // All 3 sensor pairs
    private final DualBallDetector sensorPair1;  // At spindexer intake slot
    private final DualBallDetector sensorPair2;  // Middle belt
    private final DualBallDetector sensorPair3;  // Furthest out

    // State tracking
    private CatalogingCases state = CatalogingCases.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Pre-scanned ball colors (assigned in SCANNING_ALL_SENSORS)
    private BallColor ball1Color = BallColor.None;  // Will go to shooter (fast) or slot 2 (sorted)
    private BallColor ball2Color = BallColor.None;  // Will go to slot 1 (sorted) or slot 0 (fast)
    private BallColor ball3Color = BallColor.None;  // Will go to slot 0 (sorted) or slot 2 (fast)

    public CatalogManager(
            Spindexer spindexer,
            Intake intake,
            Telemetry telemetry,
            DualBallDetector sensorPair1,
            DualBallDetector sensorPair2,
            DualBallDetector sensorPair3
    ) {
        this.spindexer = spindexer;
        this.intake = intake;
        this.telemetry = telemetry;
        this.sensorPair1 = sensorPair1;
        this.sensorPair2 = sensorPair2;
        this.sensorPair3 = sensorPair3;
    }

    /**
     * Set the shooting sequence manager reference for mode checking
     */
    public void setSequenceManager(ShootingSequenceManager sequenceManager) {
        this.sequenceManager = sequenceManager;
    }

    /**
     * Check if we're in fast shooting mode
     */
    private boolean isFastMode() {
        return sequenceManager != null && sequenceManager.getMode() == ShootingMode.Fast;
    }

    public CatalogingCases getState() {
        return state;
    }

    /**
     * Check if catalogging is currently active (not idle)
     */
    public boolean isActive() {
        return state != CatalogingCases.IDLE;
    }

    /**
     * Manual trigger for catalogging (Y button)
     * Works even if not all 3 sensors detect balls
     */
    public void initiateCataloging() {
        if (state != CatalogingCases.IDLE) {
            return; // Already running
        }
        state = CatalogingCases.SCANNING_ALL_SENSORS;
        stateTimer.reset();
    }

    /**
     * Main update loop - call every cycle
     */
    public void update() {
        // Always update sensors for detection
        sensorPair1.update();
        sensorPair2.update();
        sensorPair3.update();

        switch (state) {
            case IDLE:
                handleIdleState();
                break;

            case SCANNING_ALL_SENSORS:
                handleScanningAllSensors();
                break;

            // ========== SORTED MODE STATES ==========
            case POSITION_TO_300:
                handlePositionTo300();
                break;

            case WAIT_POSITION:
                handleWaitPosition();
                break;

            case INTAKE_BALL_1:
                handleIntakeBall1();
                break;

            case WAIT_BALL_1:
                handleWaitBall1();
                break;

            case ROTATE_TO_240:
                handleRotateTo240();
                break;

            case WAIT_ROTATION_240:
                handleWaitRotation240();
                break;

            case INTAKE_BALL_2:
                handleIntakeBall2();
                break;

            case WAIT_BALL_2:
                handleWaitBall2();
                break;

            case ROTATE_TO_180:
                handleRotateTo180();
                break;

            case WAIT_ROTATION_180:
                handleWaitRotation180();
                break;

            case INTAKE_BALL_3:
                handleIntakeBall3();
                break;

            case WAIT_BALL_3:
                handleWaitBall3();
                break;

            // ========== FAST MODE STATES ==========
            case FAST_POSITION_TO_240:
                handleFastPositionTo240();
                break;

            case FAST_WAIT_POSITION:
                handleFastWaitPosition();
                break;

            case FAST_FLIP_AND_INTAKE:
                handleFastFlipAndIntake();
                break;

            case FAST_WAIT_FLIP:
                handleFastWaitFlip();
                break;

            case FAST_ROTATE_TO_180:
                handleFastRotateTo180();
                break;

            case FAST_WAIT_ROTATION:
                handleFastWaitRotation();
                break;

            case FAST_INTAKE_BALL:
                handleFastIntakeBall();
                break;

            case FAST_WAIT_BALL:
                handleFastWaitBall();
                break;

            case COMPLETE:
                handleComplete();
                break;
        }
    }

    // ==================== State Handlers ====================

    private void handleIdleState() {
        // Don't auto-trigger if spindexer is busy rotating
        if (!spindexer.isRotationIdle()) {
            return;
        }

        // Don't auto-trigger if spindexer flipper is still active
        if (!spindexer.isReadyToFlip()) {
            return;
        }

        // Don't auto-trigger if spindexer already has balls
        if (SpindexerPattern.getBallCount() > 0) {
            return;
        }

        // Ensure spindexer is at 300 degrees when empty (ready for intake)
        if (spindexer.getCurrentDegrees() != SpindexerConstants.EMPTY_RESET_DEGREES) {
            spindexer.setDegree(SpindexerConstants.EMPTY_RESET_DEGREES);
            return;
        }

        // Check for auto-trigger: all 3 sensors detect balls
        if (allSensorsDetectBalls()) {
            state = CatalogingCases.SCANNING_ALL_SENSORS;
            stateTimer.reset();
        }
    }

    private void handleScanningAllSensors() {
        // Read all 3 sensor pairs and store colors
        scanAndAssignSlots();

        // Branch based on shooting mode
        if (isFastMode()) {
            // Fast mode: start at 240 to load first ball directly into shooter
            state = CatalogingCases.FAST_POSITION_TO_240;
        } else {
            // Sorted mode: start at 300 to fill all slots
            state = CatalogingCases.POSITION_TO_300;
        }
        stateTimer.reset();
    }

    private void handlePositionTo300() {
        // Set spindexer to starting position (300 degrees = slot 2 at intake)
        spindexer.setDegree(SpindexerConstants.EMPTY_RESET_DEGREES);
        state = CatalogingCases.WAIT_POSITION;
        stateTimer.reset();
    }

    private void handleWaitPosition() {
        // Wait for spindexer to reach position or timeout
        if (spindexer.isRotationIdle() ||
                stateTimer.seconds() > RobotConstants.Cataloging.POSITION_TIMEOUT_SECONDS) {
            state = CatalogingCases.INTAKE_BALL_1;
            stateTimer.reset();
        }
    }

    private void handleIntakeBall1() {
        // Check if ball is already at intake slot (sensor pair 1)
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent) {
            // Ball already at intake - skip intake, assign color and rotate
            intake.stopIntake();
            SpindexerPattern.setBallInSlotX(2, ball1Color);
            state = CatalogingCases.ROTATE_TO_240;
            stateTimer.reset();
            return;
        }

        // No ball at intake yet - start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_1;
        stateTimer.reset();
    }

    private void handleWaitBall1() {
        // Check if ball arrived at intake slot
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent || stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
            intake.stopIntake();
            // Assign ball 1 color to slot 2 (at 300 deg position)
            SpindexerPattern.setBallInSlotX(2, ball1Color);
            state = CatalogingCases.ROTATE_TO_240;
            stateTimer.reset();
        }
    }

    private void handleRotateTo240() {
        // Rotate CCW from 300 to 240 (opens slot 1 for next ball)
        spindexer.rotateCCW();
        state = CatalogingCases.WAIT_ROTATION_240;
        stateTimer.reset();
    }

    private void handleWaitRotation240() {
        // Wait for rotation to complete or timeout
        if (spindexer.isRotationIdle() ||
                stateTimer.seconds() > RobotConstants.Cataloging.ROTATION_TIMEOUT_SECONDS) {
            state = CatalogingCases.INTAKE_BALL_2;
            stateTimer.reset();
        }
    }

    private void handleIntakeBall2() {
        // Check if ball is already at intake slot (sensor pair 1)
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent) {
            // Ball already at intake - skip intake, assign color and rotate
            intake.stopIntake();
            SpindexerPattern.setBallInSlotX(1, ball2Color);
            state = CatalogingCases.ROTATE_TO_180;
            stateTimer.reset();
            return;
        }

        // No ball at intake yet - start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_2;
        stateTimer.reset();
    }

    private void handleWaitBall2() {
        // Check if ball arrived at intake slot
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent || stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
            intake.stopIntake();
            // Assign ball 2 color to slot 1 (at 240 deg position)
            SpindexerPattern.setBallInSlotX(1, ball2Color);
            state = CatalogingCases.ROTATE_TO_180;
            stateTimer.reset();
        }
    }

    private void handleRotateTo180() {
        // Rotate CCW from 240 to 180 (opens slot 0 for last ball)
        spindexer.rotateCCW();
        state = CatalogingCases.WAIT_ROTATION_180;
        stateTimer.reset();
    }

    private void handleWaitRotation180() {
        // Wait for rotation to complete or timeout
        if (spindexer.isRotationIdle() ||
                stateTimer.seconds() > RobotConstants.Cataloging.ROTATION_TIMEOUT_SECONDS) {
            state = CatalogingCases.INTAKE_BALL_3;
            stateTimer.reset();
        }
    }

    private void handleIntakeBall3() {
        // Check if ball is already at intake slot (sensor pair 1)
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent) {
            // Ball already at intake - skip intake, assign color and complete
            intake.stopIntake();
            SpindexerPattern.setBallInSlotX(0, ball3Color);
            state = CatalogingCases.COMPLETE;
            return;
        }

        // No ball at intake yet - start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_3;
        stateTimer.reset();
    }

    private void handleWaitBall3() {
        // Check if ball arrived at intake slot
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent || stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
            intake.stopIntake();
            // Assign ball 3 color to slot 0 (at 180 deg position)
            SpindexerPattern.setBallInSlotX(0, ball3Color);
            state = CatalogingCases.COMPLETE;
        }
    }

    private void handleComplete() {
        // Ensure intake is stopped
        intake.stopIntake();

        // Reset ball colors for next cycle
        ball1Color = BallColor.None;
        ball2Color = BallColor.None;
        ball3Color = BallColor.None;

        // Return to idle
        state = CatalogingCases.IDLE;
    }

    // ==================== Fast Mode Handlers ====================

    private void handleFastPositionTo240() {
        // Set spindexer to 240 degrees (slot 1/shooter slot at intake)
        spindexer.setDegree(240);
        state = CatalogingCases.FAST_WAIT_POSITION;
        stateTimer.reset();
    }

    private void handleFastWaitPosition() {
        // Wait for spindexer to reach position
        if (spindexer.isRotationIdle() ||
                stateTimer.seconds() > RobotConstants.Cataloging.POSITION_TIMEOUT_SECONDS) {
            state = CatalogingCases.FAST_FLIP_AND_INTAKE;
            stateTimer.reset();
        }
    }

    private void handleFastFlipAndIntake() {
        // Simultaneously flip spindexer and run intake
        // Ball 1 goes directly into shooter via the flip
        spindexer.triggerFlick();
        intake.runIntake();
        state = CatalogingCases.FAST_WAIT_FLIP;
        stateTimer.reset();
    }

    private void handleFastWaitFlip() {
        // Wait for spindexer flipper to return to idle (ball is now in shooter)
        if (spindexer.isReadyToFlip()) {
            // Stop intake once flip is complete
            intake.stopIntake();
            // Ball 1 went to shooter, not tracked in spindexer pattern
            // (It's ready to be shot immediately)
            state = CatalogingCases.FAST_ROTATE_TO_180;
            stateTimer.reset();
        }
    }

    private void handleFastRotateTo180() {
        // Rotate to 180 degrees (slot 0 at intake)
        spindexer.rotateCCW();
        state = CatalogingCases.FAST_WAIT_ROTATION;
        stateTimer.reset();
    }

    private void handleFastWaitRotation() {
        // Wait for rotation to complete
        if (spindexer.isRotationIdle() ||
                stateTimer.seconds() > RobotConstants.Cataloging.ROTATION_TIMEOUT_SECONDS) {
            state = CatalogingCases.FAST_INTAKE_BALL;
            stateTimer.reset();
        }
    }

    private void handleFastIntakeBall() {
        // Check if ball is already at intake slot
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent) {
            // Ball already at intake - assign and complete
            intake.stopIntake();
            SpindexerPattern.setBallInSlotX(0, ball2Color);
            state = CatalogingCases.COMPLETE;
            return;
        }

        // Run intake to bring in ball 2
        intake.runIntake();
        state = CatalogingCases.FAST_WAIT_BALL;
        stateTimer.reset();
    }

    private void handleFastWaitBall() {
        // Wait for ball to arrive
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        if (r1.ballPresent || stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
            intake.stopIntake();
            // Ball 2 enters slot 0 (at 180 deg position)
            SpindexerPattern.setBallInSlotX(0, ball2Color);
            state = CatalogingCases.COMPLETE;
        }
    }

    // ==================== Helper Methods ====================

    /**
     * Check if all 3 sensor pairs detect balls
     */
    private boolean allSensorsDetectBalls() {
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        DualBallDetector.Result r2 = sensorPair2.detectBall();
        DualBallDetector.Result r3 = sensorPair3.detectBall();

        return r1.ballPresent && r2.ballPresent && r3.ballPresent;
    }

    /**
     * Read all 3 sensor pairs and store colors for pre-emptive assignment
     *
     * Mapping:
     *   - Pair1 (closest to spindexer) -> ball enters first -> Slot 2
     *   - Pair2 (middle) -> ball enters second -> Slot 1
     *   - Pair3 (furthest) -> ball enters last -> Slot 0
     */
    private void scanAndAssignSlots() {
        DualBallDetector.Result r1 = sensorPair1.detectBall();
        DualBallDetector.Result r2 = sensorPair2.detectBall();
        DualBallDetector.Result r3 = sensorPair3.detectBall();

        // Store colors - will be assigned to slots as balls enter
        ball1Color = r1.ballPresent ? r1.color : BallColor.None;
        ball2Color = r2.ballPresent ? r2.color : BallColor.None;
        ball3Color = r3.ballPresent ? r3.color : BallColor.None;
    }

    /**
     * Get status string for telemetry
     */
    public String getStatus() {
        if (state == CatalogingCases.IDLE) {
            return "IDLE";
        }
        return String.format("%s [B1:%s B2:%s B3:%s]",
                state.toString(),
                ball1Color,
                ball2Color,
                ball3Color
        );
    }
}
