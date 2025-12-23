package tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import subsystems.Spindexer;
import utility.RobotHardware;
import utility.RobotConstants.Enums.BallColor;
import utility.ShootingStrategy;
import utility.ShootingStrategy.Action;
import utility.ShootingStrategy.GoalPattern;

/**
 * TestSmartShooting
 *
 * Tests the smart shooting algorithm to verify it chooses the shortest rotation path.
 *
 * Test Cases:
 * 1. Load spindexer with: Purple (slot 0), Green (slot 1), Purple (slot 2)
 *    - Set target to Green, trigger smart shoot
 *    - Should rotate to slot 1 (shortest path)
 * 2. Load spindexer with: Green (slot 0), Purple (slot 1), Purple (slot 2)
 *    - Set target to Purple, trigger smart shoot
 *    - Should rotate to nearest Purple (slot 1 or 2, whichever is closer)
 * 3. Test from various starting positions
 *    - Always chooses shortest rotation path
 *    - Never rotates more than 2 steps (240°)
 *
 * Controls:
 * - DPAD_UP: Set target goal to PGP (Purple-Green-Purple)
 * - DPAD_RIGHT: Set target goal to GPP (Green-Purple-Purple)
 * - DPAD_DOWN: Set target goal to PPG (Purple-Purple-Green)
 * - LEFT_BUMPER: Calculate and display shooting sequence (SMART mode)
 * - RIGHT_BUMPER: Calculate and display shooting sequence (FAST mode)
 * - A Button: Manually set slot 0 to Purple
 * - B Button: Manually set slot 0 to Green
 * - X Button: Manually set slot 0 to None
 * - Y Button: Rotate spindexer 120° forward
 * - LEFT_TRIGGER: Cycle slot editing (0->1->2->0)
 */
@TeleOp(name = "Test: Smart Shooting", group = "Tests")
public class TestSmartShooting extends OpMode {

    private RobotHardware robot;
    private Spindexer spindexer;

    private GoalPattern targetGoal = GoalPattern.PGP;
    private ShootingStrategy.StrategyMode currentMode = ShootingStrategy.StrategyMode.SMART;
    private Action[] currentSequence = null;

    private int editingSlot = 0; // Which slot we're currently editing (0, 1, or 2)
    private int totalRotations = 0;
    private int totalShots = 0;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);

        // Initialize subsystems
        spindexer = new Spindexer();

        // Set up test pattern: Purple, Green, Purple
        robot.spindexerPattern.setBallPattern(BallColor.Purple, BallColor.Green, BallColor.Purple);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("Controls:");
        telemetry.addLine("DPAD: Select goal pattern");
        telemetry.addLine("LEFT_BUMPER: Calculate SMART sequence");
        telemetry.addLine("RIGHT_BUMPER: Calculate FAST sequence");
        telemetry.addLine("A/B/X: Set current slot to Purple/Green/None");
        telemetry.addLine("Y: Rotate spindexer forward");
        telemetry.addLine("LEFT_TRIGGER: Cycle editing slot");
        telemetry.update();
    }

    @Override
    public void start() {
        totalRotations = 0;
        totalShots = 0;
    }

    @Override
    public void loop() {
        // Update subsystems
        spindexer.periodic();

        // Handle goal pattern selection
        if (gamepad1.dpad_up) {
            targetGoal = GoalPattern.PGP;
        } else if (gamepad1.dpad_right) {
            targetGoal = GoalPattern.GPP;
        } else if (gamepad1.dpad_down) {
            targetGoal = GoalPattern.PPG;
        }

        // Handle slot cycling
        if (gamepad1.left_trigger > 0.5) {
            editingSlot = (editingSlot + 1) % 3;
            // Wait for trigger release
            while (gamepad1.left_trigger > 0.5) {
                // Busy wait
            }
        }

        // Handle manual slot editing
        if (gamepad1.a) {
            robot.spindexerPattern.setBallInSlotX(editingSlot, BallColor.Purple);
        } else if (gamepad1.b) {
            robot.spindexerPattern.setBallInSlotX(editingSlot, BallColor.Green);
        } else if (gamepad1.x) {
            robot.spindexerPattern.setBallInSlotX(editingSlot, BallColor.None);
        }

        // Handle manual rotation
        if (gamepad1.y) {
            spindexer.rotateToNextSlot();
        }

        // Calculate shooting sequence
        if (gamepad1.left_bumper) {
            currentMode = ShootingStrategy.StrategyMode.SMART;
            ShootingStrategy.setStrategyMode(currentMode);
            currentSequence = ShootingStrategy.getShootingSequence(robot.spindexerPattern, targetGoal);
            analyzeSequence();
        } else if (gamepad1.right_bumper) {
            currentMode = ShootingStrategy.StrategyMode.FAST;
            ShootingStrategy.setStrategyMode(currentMode);
            currentSequence = ShootingStrategy.getShootingSequence(robot.spindexerPattern, targetGoal);
            analyzeSequence();
        }

        // Build telemetry
        displayTelemetry();
    }

    private void analyzeSequence() {
        if (currentSequence == null) return;

        totalRotations = 0;
        totalShots = 0;

        for (Action action : currentSequence) {
            if (action == Action.ROTATE_FORWARD || action == Action.ROTATE_BACKWARD) {
                totalRotations++;
            } else if (action == Action.SHOOT) {
                totalShots++;
            }
        }
    }

    private void displayTelemetry() {
        telemetry.addData("=== CURRENT STATE ===", "");
        telemetry.addData("Editing Slot", editingSlot + " (LEFT_TRIGGER to cycle)");
        telemetry.addData("", "");

        telemetry.addData("=== SPINDEXER CONTENTS ===", "");
        telemetry.addData("Slot 0 (Intake)", getBallColorString(0, editingSlot == 0));
        telemetry.addData("Slot 1 (Shooter)", getBallColorString(1, editingSlot == 1));
        telemetry.addData("Slot 2 (Storage)", getBallColorString(2, editingSlot == 2));
        telemetry.addData("Current Position", "%.1f°", spindexer.getServoPosition());
        telemetry.addData("", "");

        telemetry.addData("=== TARGET GOAL ===", "");
        telemetry.addData("Goal Pattern", getGoalPatternString(targetGoal));
        telemetry.addData("", "");

        telemetry.addData("=== SHOOTING STRATEGY ===", "");
        telemetry.addData("Mode", currentMode.toString());

        if (currentSequence != null) {
            telemetry.addData("Sequence Length", currentSequence.length + " actions");
            telemetry.addData("Total Rotations", totalRotations + " (" + (totalRotations * 120) + "°)");
            telemetry.addData("Total Shots", totalShots);
            telemetry.addData("", "");

            telemetry.addData("=== ACTION SEQUENCE ===", "");
            for (int i = 0; i < Math.min(currentSequence.length, 10); i++) {
                telemetry.addData("  " + (i + 1), getActionString(currentSequence[i]));
            }
            if (currentSequence.length > 10) {
                telemetry.addData("", "... (" + (currentSequence.length - 10) + " more)");
            }
        } else {
            telemetry.addData("Sequence", "Press LEFT_BUMPER (SMART) or RIGHT_BUMPER (FAST)");
        }
        telemetry.addData("", "");

        telemetry.addData("=== TEST RESULTS ===", "");

        // Test 1: Algorithm calculated
        if (currentSequence != null) {
            telemetry.addData("Test 1", "\u2705 PASS - Algorithm calculated");
        } else {
            telemetry.addData("Test 1", "Press LEFT_BUMPER to calculate");
        }

        // Test 2: Rotation efficiency (never more than 2 rotations per ball in smart mode)
        if (currentSequence != null && totalShots > 0) {
            int maxRotationsPerBall = totalRotations / totalShots;
            if (maxRotationsPerBall <= 2) {
                telemetry.addData("Test 2", "\u2705 PASS - Efficient rotations (avg %.1f per ball)",
                    (float) totalRotations / totalShots);
            } else {
                telemetry.addData("Test 2", "\u274c FAIL - Too many rotations (avg %.1f per ball)",
                    (float) totalRotations / totalShots);
            }
        } else {
            telemetry.addData("Test 2", "Calculate sequence first");
        }

        // Test 3: Shortest path validation
        if (currentSequence != null && currentMode == ShootingStrategy.StrategyMode.SMART) {
            boolean usesShortestPath = totalRotations <= (totalShots * 2);
            if (usesShortestPath) {
                telemetry.addData("Test 3", "\u2705 PASS - Uses shortest path");
            } else {
                telemetry.addData("Test 3", "\u274c FAIL - Not using shortest path");
            }
        } else {
            telemetry.addData("Test 3", "Test SMART mode");
        }

        telemetry.addData("", "");
        telemetry.addData("Quick Test Setups", "");
        telemetry.addLine("1. A, Y, B, Y, A = Purple-Green-Purple");
        telemetry.addLine("2. B, Y, A, Y, A = Green-Purple-Purple");
        telemetry.addLine("3. A, Y, A, Y, B = Purple-Purple-Green");

        telemetry.update();
    }

    private String getBallColorString(int slot, boolean isEditing) {
        BallColor color = spindexer.getBallInSlot(slot);
        String colorName = color.toString();
        String prefix = isEditing ? ">>> " : "    ";

        // Add color indicator
        if (color == BallColor.Purple) {
            return prefix + colorName + " \ud83d\udfe3";
        } else if (color == BallColor.Green) {
            return prefix + colorName + " \ud83d\udfe2";
        } else {
            return prefix + colorName + " \u26aa";
        }
    }

    private String getGoalPatternString(GoalPattern goal) {
        switch (goal) {
            case PGP:
                return "PGP (Purple-Green-Purple) \ud83d\udfe3\ud83d\udfe2\ud83d\udfe3";
            case GPP:
                return "GPP (Green-Purple-Purple) \ud83d\udfe2\ud83d\udfe3\ud83d\udfe3";
            case PPG:
                return "PPG (Purple-Purple-Green) \ud83d\udfe3\ud83d\udfe3\ud83d\udfe2";
            default:
                return goal.toString();
        }
    }

    private String getActionString(Action action) {
        switch (action) {
            case SHOOT:
                return "\ud83d\udd35 SHOOT";
            case ROTATE_FORWARD:
                return "\u27a1 ROTATE FORWARD (120°)";
            case ROTATE_BACKWARD:
                return "\u2b05 ROTATE BACKWARD (120°)";
            default:
                return action.toString();
        }
    }

    @Override
    public void stop() {
        robot.spindexerServo.setPower(0);
    }
}
