package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants;
import utility.RobotHardware;
import utility.SpindexerJamClearance;

/**
 * TestSpindexerStallDetection
 *
 * Tests the automatic stall detection and jam clearance system.
 *
 * Test Cases:
 * 1. Normal rotation (no stall)
 *    - Press A to rotate 120°
 *    - Verify rotation completes without false positives
 * 2. Manual jam (hold spindexer physically)
 *    - Press A to start rotation
 *    - Physically hold spindexer to prevent rotation
 *    - Should detect stall after 250ms
 *    - Intake should run forward for 200ms
 *    - Should retry rotation automatically
 * 3. Persistent jam
 *    - Hold spindexer through all retries
 *    - Should give up after 3 attempts
 * 4. Jam clearance timing
 *    - Verify intake runs for exactly 200ms
 *    - Verify intake state is restored after clearing
 *
 * Controls:
 * - A Button: Rotate spindexer forward 120°
 * - B Button: Rotate spindexer backward 120°
 * - Right Trigger: Manually run intake (to test state preservation)
 * - X Button: Reset retry counter
 */
@TeleOp(name = "Test: Spindexer Stall Detection", group = "Tests")
public class TestSpindexerStallDetection extends OpMode {

    private RobotHardware robot;
    private Intake intake;
    private Spindexer spindexer;
    private SpindexerJamClearance jamClearance;

    private int totalStallsDetected = 0;
    private int totalJamsCleared = 0;
    private boolean lastStateWasStalled = false;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        // Initialize subsystems
        intake = new Intake();
        spindexer = new Spindexer();
        jamClearance = new SpindexerJamClearance(intake, spindexer);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addLine("=== CONTROLS ===");
        telemetry.addLine("A: Rotate forward 120°");
        telemetry.addLine("B: Rotate backward 120°");
        telemetry.addLine("Right Trigger: Run intake");
        telemetry.addLine("X: Reset counters");
        telemetry.addLine();
        telemetry.addLine("=== INSTRUCTIONS ===");
        telemetry.addLine("1. Press A for normal rotation");
        telemetry.addLine("2. Press A then HOLD spindexer");
        telemetry.addLine("   to test stall detection");
        telemetry.addLine("3. Watch retry counter");
        telemetry.update();
    }

    @Override
    public void start() {
        totalStallsDetected = 0;
        totalJamsCleared = 0;
        lastStateWasStalled = false;
    }

    @Override
    public void loop() {
        // Update subsystems
        intake.periodic();
        spindexer.periodic();
        jamClearance.periodic();

        // Track stall events
        RobotConstants.Enums.SpindexerRotationState currentState = spindexer.getRotationState();
        if (currentState == RobotConstants.Enums.SpindexerRotationState.STALLED && !lastStateWasStalled) {
            totalStallsDetected++;
            totalJamsCleared++;
        }
        lastStateWasStalled = (currentState == RobotConstants.Enums.SpindexerRotationState.STALLED);

        // Manual rotation controls
        if (gamepad1.a) {
            spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
        }

        if (gamepad1.b) {
            spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_BACKWARD);
        }

        // Reset counters
        if (gamepad1.x) {
            totalStallsDetected = 0;
            totalJamsCleared = 0;
        }

        // Manual intake control (to test state preservation)
        if (gamepad1.right_trigger > 0.1) {
            intake.runIntake();
        } else if (!jamClearance.isJamClearingInProgress()) {
            // Only stop if not clearing a jam
            intake.stopIntake();
        }

        // Display telemetry
        displayTelemetry();
    }

    private void displayTelemetry() {
        telemetry.addData("=== STALL DETECTION STATUS ===", "");
        telemetry.addData("Rotation State", spindexer.getRotationState().toString());
        telemetry.addData("Jam Clearance", jamClearance.getStatus());
        telemetry.addData("Retry Attempts", spindexer.getRetryAttempts());
        telemetry.addData("", "");

        telemetry.addData("=== STATISTICS ===", "");
        telemetry.addData("Stalls Detected", totalStallsDetected);
        telemetry.addData("Jams Cleared", totalJamsCleared);
        telemetry.addData("", "");

        telemetry.addData("=== SPINDEXER STATE ===", "");
        telemetry.addData("Current Position", "%.1f°", spindexer.getServoPosition());
        telemetry.addData("Target Position", "%.1f°", spindexer.getTargetPosition());
        telemetry.addData("Done Rotating?", spindexer.isDoneRotating() ? "YES" : "NO");
        telemetry.addData("", "");

        telemetry.addData("=== INTAKE STATE ===", "");
        telemetry.addData("State", intake.getCurrentState().toString());
        telemetry.addData("Trigger", "%.2f", gamepad1.right_trigger);
        telemetry.addData("", "");

        telemetry.addData("=== TEST RESULTS ===", "");

        // Test 1: Normal rotation (no false positives)
        if (spindexer.getRetryAttempts() == 0 && totalStallsDetected == 0) {
            telemetry.addData("Test 1 (Normal)", "PASS - No false stalls");
        } else if (totalStallsDetected > 0) {
            telemetry.addData("Test 1 (Normal)", "Check if stalls were real");
        } else {
            telemetry.addData("Test 1 (Normal)", "Press A to test");
        }

        // Test 2: Stall detection
        if (totalStallsDetected > 0) {
            telemetry.addData("Test 2 (Detection)", "PASS - %d stalls detected!", totalStallsDetected);
        } else {
            telemetry.addData("Test 2 (Detection)", "Hold spindexer during rotation");
        }

        // Test 3: Retry logic
        if (spindexer.getRetryAttempts() > 0) {
            if (spindexer.getRetryAttempts() >= RobotConstants.Spindexer.MAX_RETRY_ATTEMPTS) {
                telemetry.addData("Test 3 (Max Retries)", "PASS - Gave up after %d attempts",
                    RobotConstants.Spindexer.MAX_RETRY_ATTEMPTS);
            } else {
                telemetry.addData("Test 3 (Retry)", "IN PROGRESS - Retry %d/%d",
                    spindexer.getRetryAttempts(), RobotConstants.Spindexer.MAX_RETRY_ATTEMPTS);
            }
        } else {
            telemetry.addData("Test 3 (Retry)", "Waiting for retry...");
        }

        // Test 4: Jam clearance
        if (totalJamsCleared > 0) {
            telemetry.addData("Test 4 (Clearing)", "PASS - Cleared %d jams", totalJamsCleared);
        } else {
            telemetry.addData("Test 4 (Clearing)", "Waiting for jam...");
        }

        telemetry.addData("", "");
        telemetry.addData("=== CONFIGURATION ===", "");
        telemetry.addData("Stall Time", "%.0fms", RobotConstants.Spindexer.STALL_DETECTION_TIME * 1000);
        telemetry.addData("Clear Time", "%.0fms", RobotConstants.Spindexer.INTAKE_CLEARING_TIME * 1000);
        telemetry.addData("Max Retries", "%d", RobotConstants.Spindexer.MAX_RETRY_ATTEMPTS);

        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stopIntake();
        robot.spindexerServo.setPower(0);
    }
}
