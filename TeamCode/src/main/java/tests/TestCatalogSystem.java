package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.ColorSensorSubsytem;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.CatalogManager;
import utility.RobotHardware;
import utility.RobotConstants.Enums.BallColor;

/**
 * TestCatalogSystem
 *
 * Tests the automatic ball cataloging system.
 *
 * Test Cases:
 * 1. Place ball in intake, press trigger, release trigger
 *    - Should catalog ball color
 *    - Should rotate spindexer 120°
 * 2. Fill all 3 spindexer slots, try to add 4th ball
 *    - Intake should reverse
 *    - Extra ball expelled
 * 3. Catalog balls of different colors
 *    - Each ball tracked separately
 *
 * Controls:
 * - Right Trigger: Run intake
 * - Release: Catalog ball
 * - A Button: Manually reset spindexer pattern (for testing)
 * - B Button: Manually rotate spindexer 120° (for testing)
 */
@TeleOp(name = "Test: Catalog System", group = "Tests")
public class TestCatalogSystem extends OpMode {

    private RobotHardware robot;
    private CatalogManager catalogManager;
    private Intake intake;
    private Spindexer spindexer;
    private ColorSensorSubsytem colorSensor;

    private int catalogCount = 0;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        // Initialize subsystems
        intake = new Intake();
        spindexer = new Spindexer();
        colorSensor = new ColorSensorSubsytem();

        // Initialize catalog manager
        catalogManager = new CatalogManager(robot.intakeSensor, spindexer, intake);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("Right Trigger: Run intake");
        telemetry.addLine("Release: Catalog ball");
        telemetry.addLine("A: Reset spindexer pattern");
        telemetry.addLine("B: Manual rotate 120°");
        telemetry.update();
    }

    @Override
    public void start() {
        catalogCount = 0;
    }

    @Override
    public void loop() {
        // Update subsystems
        intake.periodic();
        spindexer.periodic();
        colorSensor.periodic();

        // Manual controls for testing
        if (gamepad1.a) {
            robot.spindexerPattern.emptyBallPattern();
            catalogCount = 0;
        }

        if (gamepad1.b) {
            spindexer.rotateToNextSlot();
        }

        // Intake control
        boolean intakeActive = gamepad1.right_trigger > 0.1;
        if (intakeActive) {
            intake.runIntake();
        } else {
            intake.stopIntake();
        }

        // Update catalog manager
        boolean wasCataloging = catalogManager.isCataloging();
        catalogManager.update(intakeActive);

        // Track catalog count
        if (!wasCataloging && catalogManager.isCataloging()) {
            catalogCount++;
        }

        // Build telemetry
        displayTelemetry();
    }

    private void displayTelemetry() {
        telemetry.addData("=== CATALOG STATUS ===", "");
        telemetry.addData("Status", catalogManager.getStatus());
        telemetry.addData("Total Cataloged", catalogCount);
        telemetry.addData("", "");

        telemetry.addData("=== SPINDEXER STATE ===", "");
        telemetry.addData("Full?", spindexer.isFull() ? "YES (3/3)" : "NO");
        telemetry.addData("Current Position", "%.1f°", spindexer.getServoPosition());
        telemetry.addData("Target Position", "%.1f°", spindexer.getTargetPosition());
        telemetry.addData("Done Rotating?", spindexer.isDoneRotating() ? "YES" : "NO");
        telemetry.addData("", "");

        telemetry.addData("=== BALL SLOTS ===", "");
        telemetry.addData("Slot 0 (Intake)", getBallColorString(0));
        telemetry.addData("Slot 1 (Shooter)", getBallColorString(1));
        telemetry.addData("Slot 2 (Storage)", getBallColorString(2));
        telemetry.addData("", "");

        telemetry.addData("=== COLOR SENSOR ===", "");
        BallColor currentColor = ColorSensorSubsytem.getBallColor();
        telemetry.addData("Detected Color", currentColor.toString());
        telemetry.addData("RGB Values", colorSensor.getColorDataString());
        telemetry.addData("", "");

        telemetry.addData("=== INTAKE STATE ===", "");
        telemetry.addData("State", intake.getCurrentState().toString());
        telemetry.addData("Trigger", "%.2f", gamepad1.right_trigger);
        telemetry.addData("", "");

        telemetry.addData("=== TEST RESULTS ===", "");

        // Test 1: Ball Detection
        if (currentColor != BallColor.None) {
            telemetry.addData("Test 1", "PASS - Ball detected!");
        } else {
            telemetry.addData("Test 1", "Waiting for ball...");
        }

        // Test 2: Rotation after catalog
        if (catalogCount > 0) {
            telemetry.addData("Test 2", "PASS - Cataloged %d times", catalogCount);
        } else {
            telemetry.addData("Test 2", "Release intake to catalog");
        }

        // Test 3: Full spindexer check
        if (spindexer.isFull()) {
            telemetry.addData("Test 3", "PASS - Spindexer full! Try adding 4th ball");
        } else {
            telemetry.addData("Test 3", "Add more balls to fill spindexer");
        }

        // Test 4: Reverse when full
        if (catalogManager.isReversing()) {
            telemetry.addData("Test 4", "PASS - Reversing intake (spindexer full!)");
        }

        telemetry.update();
    }

    private String getBallColorString(int slot) {
        BallColor color = spindexer.getBallInSlot(slot);
        String colorName = color.toString();

        // Add color indicator
        if (color == BallColor.Purple) {
            return colorName + " \ud83d\udfe3"; // Purple circle emoji
        } else if (color == BallColor.Green) {
            return colorName + " \ud83d\udfe2"; // Green circle emoji
        } else {
            return colorName + " \u26aa"; // White circle emoji
        }
    }

    @Override
    public void stop() {
        intake.stopIntake();
        robot.spindexerServo.setPower(0);
    }
}
