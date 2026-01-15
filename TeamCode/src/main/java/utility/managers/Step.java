package utility.managers;

import java.util.function.BooleanSupplier;

/**
 * A single step in a state machine sequence.
 * Steps can either execute immediately or wait for a condition/time.
 *
 * Usage example:
 * <pre>
 * Step[] sequence = {
 *     action("Fire shooter", () -> shooter.triggerShot()),
 *     wait(0.15),
 *     ifThen("No balls?", () -> ballCount == 0, 3),  // skip 3 steps if true
 *     action("Flip next", () -> spindexer.triggerFlick()),
 *     wait(0.075),
 *     loopWhile(() -> ballCount > 0, 2),  // jump back 2 steps while true
 *     done()
 * };
 * </pre>
 */
public class Step {
    public enum Type {
        ACTION,      // Execute action immediately, advance
        WAIT,        // Wait for time or condition, then advance
        CONDITIONAL, // If condition true, skip forward; else advance
        LOOP_END,    // If condition true, jump back; else advance
        DONE         // Sequence complete
    }

    public final Type type;
    public final Runnable action;           // For ACTION type
    public final double waitTime;           // For WAIT type (seconds)
    public final BooleanSupplier condition; // For CONDITIONAL/LOOP/WAIT
    public final int jumpSteps;             // Steps to skip/jump
    public final String name;               // For telemetry/debug

    private Step(Type type, String name, Runnable action, double waitTime,
                 BooleanSupplier condition, int jumpSteps) {
        this.type = type;
        this.name = name;
        this.action = action;
        this.waitTime = waitTime;
        this.condition = condition;
        this.jumpSteps = jumpSteps;
    }

    // ==================== FACTORY METHODS ====================

    /**
     * Execute an action immediately, then advance to next step.
     * @param name Description for telemetry
     * @param action The action to execute
     */
    public static Step action(String name, Runnable action) {
        return new Step(Type.ACTION, name, action, 0, null, 0);
    }

    /**
     * Wait for a fixed duration, then advance.
     * @param seconds Duration to wait
     */
    public static Step waitFor(double seconds) {
        return new Step(Type.WAIT, String.format("Wait %.2fs", seconds), null, seconds, null, 0);
    }

    /**
     * Wait until a condition becomes true, then advance.
     * @param name Description for telemetry
     * @param condition Condition to check each update
     */
    public static Step waitUntil(String name, BooleanSupplier condition) {
        return new Step(Type.WAIT, name, null, 0, condition, 0);
    }

    /**
     * Conditional skip: if condition is true, skip forward by skipSteps.
     * If false, just advance to next step.
     * @param name Description for telemetry
     * @param condition Condition to check
     * @param skipSteps How many steps to skip forward if condition is true
     */
    public static Step ifThen(String name, BooleanSupplier condition, int skipSteps) {
        return new Step(Type.CONDITIONAL, name, null, 0, condition, skipSteps);
    }

    /**
     * Loop end: if condition is true, jump back by backSteps.
     * If false, advance to next step (exit loop).
     * @param condition Continue looping while this is true
     * @param backSteps How many steps to jump back
     */
    public static Step loopWhile(BooleanSupplier condition, int backSteps) {
        return new Step(Type.LOOP_END, "Loop check", null, 0, condition, backSteps);
    }

    /**
     * Marks the end of a sequence.
     */
    public static Step done() {
        return new Step(Type.DONE, "Done", null, 0, null, 0);
    }
}
