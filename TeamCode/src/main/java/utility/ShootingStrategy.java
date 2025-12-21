package utility;

import utility.RobotConstants.Enums.BallColor;
import utility.RobotConstants.SpindexerPattern;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class ShootingStrategy {

    // Instructions for rotating balls through circular positions
    public enum Action {
        SHOOT,              // Shoot ball at outtake position (position 1)
        ROTATE_FORWARD,     // Rotate balls forward (0->1, 1->2, 2->0)
        ROTATE_BACKWARD     // Rotate balls backward (0->2, 1->0, 2->1)
    }

    public enum StrategyMode {
        SMART,      // Greedy: Match correct colors in order, then dump remaining
        FAST        // Greedy: Always shoot nearest ball regardless of color
    }

    public enum GoalPattern {
        PGP,  // Purple-Green-Purple
        GPP,  // Green-Purple-Purple
        PPG   // Purple-Purple-Green
    }

    private static class StateKey {
        final int hash;

        StateKey(BallColor b0, BallColor b1,
                 BallColor b2, GoalPattern goalPattern) {
            this.hash = (b0.ordinal() << 6) | (b1.ordinal() << 4) |
                    (b2.ordinal() << 2) | goalPattern.ordinal();
        }

        @Override
        public boolean equals(Object o) {
            return o instanceof StateKey && ((StateKey)o).hash == this.hash;
        }

        @Override
        public int hashCode() {
            return hash;
        }
    }

    private static final Map<StateKey, Action[]> STRATEGY_MAP = new HashMap<>();
    private static final int SHOOTING_POSITION = 1;  // Position 1 is outtake (shoots)

    // Default mode is SMART
    private static StrategyMode currentMode = StrategyMode.FAST;

    static {
        initializeStrategies();
    }

    /**
     * Toggles the strategy mode and re-initializes the lookup map.
     * Call this when game state changes.
     * @param mode The new mode to use (SMART or FAST)
     */
    public static void setStrategyMode(StrategyMode mode) {
        if (currentMode != mode) {
            currentMode = mode;
            STRATEGY_MAP.clear();
            initializeStrategies();
            System.out.println("Shooting Strategy switched to: " + mode);
        }
    }

    public static StrategyMode getStrategyMode() {
        return currentMode;
    }

    private static void initializeStrategies() {
        for (GoalPattern goal : GoalPattern.values()) {
            for (BallColor b0 : BallColor.values()) {
                for (BallColor b1 : BallColor.values()) {
                    for (BallColor b2 : BallColor.values()) {
                        StateKey key = new StateKey(b0, b1, b2, goal);
                        Action[] actions = computeOptimalSequence(b0, b1, b2, goal);
                        STRATEGY_MAP.put(key, actions);
                    }
                }
            }
        }
    }

    private static Action[] computeOptimalSequence(
            BallColor b0, BallColor b1,
            BallColor b2, GoalPattern goal) {

        List<Action> actions = new ArrayList<>();

        // Mutable simulation of the robot state
        BallColor[] currentBalls = {b0, b1, b2};

        if (currentMode == StrategyMode.FAST) {
            // FAST MODE: Greedy - always shoot nearest ball
            while (hasAnyBall(currentBalls)) {
                int nearestBallIndex = getNearestSlotWithAnyBall(currentBalls);
                processShot(actions, currentBalls, nearestBallIndex);
            }
        } else {
            // SMART MODE: Greedy - match correct colors in order, then dump remaining
            BallColor[] targetSequence = getTargetArray(goal);

            // Phase 1: Greedily shoot correct colors in order
            for (BallColor neededColor : targetSequence) {
                if (neededColor == BallColor.None) continue;

                // Find the NEAREST ball with the needed color
                int bestSlotIndex = getNearestSlotWithColor(currentBalls, neededColor);

                if (bestSlotIndex != -1) {
                    // Found correct color - shoot it
                    processShot(actions, currentBalls, bestSlotIndex);
                } else {
                    // No more of this color - stop trying to match pattern
                    break;
                }
            }

            // Phase 2: Greedily dump remaining balls (nearest first)
            while (hasAnyBall(currentBalls)) {
                int nearestBallIndex = getNearestSlotWithAnyBall(currentBalls);
                processShot(actions, currentBalls, nearestBallIndex);
            }
        }

        return actions.toArray(new Action[0]);
    }

    /**
     * Helper to calculate rotations, add actions, update state, and shoot.
     */
    private static void processShot(List<Action> actions, BallColor[] balls, int targetSlotIndex) {
        // 1. Calculate rotation
        int rotations = getRotationsNeeded(targetSlotIndex, SHOOTING_POSITION);

        // 2. Add Rotation Actions and update ball array state
        if (rotations > 0) {
            for (int i = 0; i < rotations; i++) {
                actions.add(Action.ROTATE_FORWARD);
                rotateArrayForward(balls);
            }
        } else if (rotations < 0) {
            for (int i = 0; i < Math.abs(rotations); i++) {
                actions.add(Action.ROTATE_BACKWARD);
                rotateArrayBackward(balls);
            }
        }

        // 3. Shoot
        actions.add(Action.SHOOT);
        balls[SHOOTING_POSITION] = BallColor.None; // Remove ball from simulation
    }

    /**
     * Finds the slot index of the requested color that requires the LEAST movement.
     */
    private static int getNearestSlotWithColor(BallColor[] balls, BallColor target) {
        int bestSlot = -1;
        int minDistance = 100; // Arbitrary high number

        for (int i = 0; i < balls.length; i++) {
            if (balls[i] == target) {
                int dist = Math.abs(getRotationsNeeded(i, SHOOTING_POSITION));
                if (dist < minDistance) {
                    minDistance = dist;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    /**
     * Finds the slot index of ANY ball that requires the LEAST movement.
     */
    private static int getNearestSlotWithAnyBall(BallColor[] balls) {
        int bestSlot = -1;
        int minDistance = 100;

        for (int i = 0; i < balls.length; i++) {
            if (balls[i] != BallColor.None) {
                int dist = Math.abs(getRotationsNeeded(i, SHOOTING_POSITION));
                if (dist < minDistance) {
                    minDistance = dist;
                    bestSlot = i;
                }
            }
        }
        return bestSlot;
    }

    private static boolean hasAnyBall(BallColor[] balls) {
        for (BallColor b : balls) {
            if (b != BallColor.None) return true;
        }
        return false;
    }

    // Helper to rotate the array in place during simulation (0->1, 1->2, 2->0)
    private static void rotateArrayForward(BallColor[] balls) {
        BallColor temp2 = balls[2];
        balls[2] = balls[1];
        balls[1] = balls[0];
        balls[0] = temp2;
    }

    // Helper to rotate the array in place during simulation (0->2, 1->0, 2->1)
    private static void rotateArrayBackward(BallColor[] balls) {
        BallColor temp0 = balls[0];
        balls[0] = balls[1];
        balls[1] = balls[2];
        balls[2] = temp0;
    }

    // Returns positive for forward, negative for backward, 0 for no rotation
    private static int getRotationsNeeded(int fromPos, int toPos) {
        if (fromPos == toPos) return 0;

        int forward = (toPos - fromPos + 3) % 3;
        if (forward == 0) forward = 3;

        int backward = (fromPos - toPos + 3) % 3;
        if (backward == 0) backward = 3;

        return forward <= backward ? forward : -backward;
    }

    private static BallColor[] getTargetArray(GoalPattern goal) {
        switch (goal) {
            case PGP: return new BallColor[]{BallColor.Purple, BallColor.Green, BallColor.Purple};
            case GPP: return new BallColor[]{BallColor.Green, BallColor.Purple, BallColor.Purple};
            case PPG: return new BallColor[]{BallColor.Purple, BallColor.Purple, BallColor.Green};
            default: return new BallColor[3];
        }
    }

    // PUBLIC API
    public static Action[] getShootingSequence(SpindexerPattern current, GoalPattern goal) {
        StateKey key = new StateKey(
                current.getBallInSlotX(0),
                current.getBallInSlotX(1),
                current.getBallInSlotX(2),
                goal
        );
        return STRATEGY_MAP.get(key);
    }

    public static Action[] getShootingSequence(SpindexerPattern current, RobotConstants.MotifPattern motifGoal) {
        GoalPattern goal = convertMotifToGoal(motifGoal);
        return getShootingSequence(current, goal);
    }

    private static GoalPattern convertMotifToGoal(RobotConstants.MotifPattern motif) {
        BallColor b0 = motif.getBallColorInSlotX(0);
        BallColor b1 = motif.getBallColorInSlotX(1);
        BallColor b2 = motif.getBallColorInSlotX(2);

        if (b0 == BallColor.Purple && b1 == BallColor.Green && b2 == BallColor.Purple) return GoalPattern.PGP;
        else if (b0 == BallColor.Green && b1 == BallColor.Purple && b2 == BallColor.Purple) return GoalPattern.GPP;
        else if (b0 == BallColor.Purple && b1 == BallColor.Purple && b2 == BallColor.Green) return GoalPattern.PPG;
        return GoalPattern.PGP;
    }
}