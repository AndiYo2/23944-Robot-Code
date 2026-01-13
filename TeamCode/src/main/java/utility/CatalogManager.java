package utility;

import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.CatalogingCases;
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
 * Monitors 3 color sensor pairs along the intake path:
 *   - Pair1: At spindexer intake slot (closest to spindexer)
 *   - Pair2: Middle belt position
 *   - Pair3: Furthest out (last to enter spindexer)
 *
 * When all 3 sensors detect balls simultaneously, automatically:
 *   1. Scan all 3 ball colors
 *   2. Pre-assign them to final spindexer slots
 *   3. Execute intake sequence: 300 -> 240 -> 180 degrees
 *
 * Ball/Slot Mapping:
 *   - Pair1 ball -> Slot 2 (enters at 300 deg)
 *   - Pair2 ball -> Slot 1 (enters at 240 deg)
 *   - Pair3 ball -> Slot 0 (enters at 180 deg)
 */
public class CatalogManager {
    // Subsystems
    private final Intake intake;
    private final Spindexer spindexer;
    private final Telemetry telemetry;

    // All 3 sensor pairs
    private final DualBallDetector sensorPair1;  // At spindexer intake slot
    private final DualBallDetector sensorPair2;  // Middle belt
    private final DualBallDetector sensorPair3;  // Furthest out

    // State tracking
    private CatalogingCases state = CatalogingCases.IDLE;
    private final ElapsedTime stateTimer = new ElapsedTime();

    // Pre-scanned ball colors (assigned in SCANNING_ALL_SENSORS)
    private BallColor ball1Color = BallColor.None;  // Will go to slot 2
    private BallColor ball2Color = BallColor.None;  // Will go to slot 1
    private BallColor ball3Color = BallColor.None;  // Will go to slot 0

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

            case COMPLETE:
                handleComplete();
                break;
        }
    }

    // ==================== State Handlers ====================

    private void handleIdleState() {
        // Don't auto-trigger if spindexer is busy
        if (!spindexer.isRotationIdle()) {
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

        // Move to position spindexer at 300 degrees
        state = CatalogingCases.POSITION_TO_300;
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
        // Start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_1;
        stateTimer.reset();
    }

    private void handleWaitBall1() {
        // Wait for ball to enter (time-based for now)
        if (stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
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
        // Start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_2;
        stateTimer.reset();
    }

    private void handleWaitBall2() {
        // Wait for ball to enter
        if (stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
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
        // Start intake motors
        intake.runIntake();
        state = CatalogingCases.WAIT_BALL_3;
        stateTimer.reset();
    }

    private void handleWaitBall3() {
        // Wait for ball to enter
        if (stateTimer.seconds() > RobotConstants.Cataloging.BALL_INTAKE_TIME) {
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

        // Return to idle - end at 180 degrees as specified
        state = CatalogingCases.IDLE;
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
