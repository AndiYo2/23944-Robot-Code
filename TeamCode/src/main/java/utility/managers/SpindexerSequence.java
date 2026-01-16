package utility.managers;

import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.ShootingMode;
import Constants.SpindexerConstants;
import subsystems.Intake;
import subsystems.Shooter;
import subsystems.Spindexer;
import utility.managers.actions.*;

import java.util.Stack;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

/**
 * Fluent builder for spindexer sequences.
 *
 * Example:
 * <pre>
 * executor = new SpindexerSequence(spindexer, shooter, intake)
 *     .fire()
 *     .repeatWhile(() -> hasBalls())
 *         .flick()
 *         .fire()
 *         .rotateToNext()
 *     .endRepeat()
 *     .resetPosition()
 *     .build();
 * </pre>
 */
public class SpindexerSequence {
    private final Spindexer spindexer;
    private final Shooter shooter;
    private final Intake intake;
    private final SpindexerExecutor executor;

    // Stack for nested repeat blocks
    private final Stack<RepeatWhileAction> repeatStack = new Stack<>();

    public SpindexerSequence(Spindexer spindexer, Shooter shooter, Intake intake) {
        this.spindexer = spindexer;
        this.shooter = shooter;
        this.intake = intake;
        this.executor = new SpindexerExecutor();
    }

    // ==================== Core Actions ====================

    /**
     * Fire the shooter.
     */
    public SpindexerSequence fire() {
        addAction(new FireAction(shooter));
        return this;
    }

    /**
     * Flick a ball from spindexer to shooter.
     */
    public SpindexerSequence flick() {
        addAction(new FlickAction(spindexer));
        return this;
    }

    /**
     * Conditionally flick a ball to shooter.
     * If condition is true, flicks and waits for extend time.
     * If condition is false, completes immediately.
     * Can be run in parallel with intake.
     */
    public SpindexerSequence possibleFlick(BooleanSupplier shouldFlick, Runnable onFlick) {
        addAction(new PossibleFlickAction(spindexer, shouldFlick, onFlick));
        return this;
    }

    /**
     * Rotate to the next closest ball.
     */
    public SpindexerSequence rotateToNext() {
        addAction(new RotateAction(spindexer));
        return this;
    }

    /**
     * Rotate to a specific ball color.
     */
    public SpindexerSequence rotateTo(BallColor color) {
        addAction(new RotateAction(spindexer, color));
        return this;
    }

    /**
     * Rotate clockwise by one position. Updates ball tracking.
     */
    public SpindexerSequence rotateCW() {
        addAction(new RotateCWAction(spindexer));
        return this;
    }

    /**
     * Rotate counter-clockwise by one position. Updates ball tracking.
     */
    public SpindexerSequence rotateCCW() {
        addAction(new RotateCCWAction(spindexer));
        return this;
    }

    /**
     * Set spindexer to a specific degree position.
     * WARNING: Does not update ball tracking - use rotateCW/CCW instead.
     */
    public SpindexerSequence setDegree(int degrees) {
        addAction(new SetDegreeAction(spindexer, degrees));
        return this;
    }

    /**
     * Reset spindexer to empty position.
     */
    public SpindexerSequence resetPosition() {
        addAction(new SetDegreeAction(spindexer, SpindexerConstants.EMPTY_RESET_DEGREES));
        return this;
    }

    /**
     * Run intake for the default duration.
     */
    public SpindexerSequence intake() {
        addAction(new IntakeAction(intake));
        return this;
    }

    /**
     * Run intake for a specific duration.
     */
    public SpindexerSequence intake(double seconds) {
        addAction(new IntakeAction(intake, seconds));
        return this;
    }

    /**
     * Wait for a duration.
     */
    public SpindexerSequence waitFor(double seconds) {
        addAction(new WaitAction(seconds));
        return this;
    }

    /**
     * Execute custom code.
     */
    public SpindexerSequence run(String name, Runnable runnable) {
        addAction(new RunnableAction(name, runnable));
        return this;
    }

    // ==================== Control Flow ====================

    /**
     * Start a repeat block that loops while the condition is true.
     * Must be closed with endRepeat().
     */
    public SpindexerSequence repeatWhile(BooleanSupplier condition) {
        RepeatWhileAction repeat = new RepeatWhileAction(condition);
        repeatStack.push(repeat);
        return this;
    }

    /**
     * End the current repeat block.
     */
    public SpindexerSequence endRepeat() {
        if (repeatStack.isEmpty()) {
            throw new IllegalStateException("endRepeat() called without matching repeatWhile()");
        }
        RepeatWhileAction repeat = repeatStack.pop();
        addAction(repeat);
        return this;
    }

    /**
     * Run multiple actions in parallel.
     * All actions start simultaneously and the parallel block completes when ALL actions are done.
     *
     * Example:
     * <pre>
     * .parallel(p -> p
     *     .flick()
     *     .run("Track", () -> updateTracking())
     * )
     * </pre>
     */
    public SpindexerSequence parallel(Consumer<ParallelBuilder> builder) {
        ParallelAction group = new ParallelAction();
        ParallelBuilder parallelBuilder = new ParallelBuilder(group, spindexer, shooter, intake);
        builder.accept(parallelBuilder);
        addAction(group);
        return this;
    }

    // ==================== Parallel Builder ====================

    /**
     * Builder for parallel action groups.
     */
    public static class ParallelBuilder {
        private final ParallelAction group;
        private final Spindexer spindexer;
        private final Shooter shooter;
        private final Intake intake;

        ParallelBuilder(ParallelAction group, Spindexer spindexer, Shooter shooter, Intake intake) {
            this.group = group;
            this.spindexer = spindexer;
            this.shooter = shooter;
            this.intake = intake;
        }

        public ParallelBuilder fire() {
            group.addAction(new FireAction(shooter));
            return this;
        }

        public ParallelBuilder flick() {
            group.addAction(new FlickAction(spindexer));
            return this;
        }

        public ParallelBuilder possibleFlick(BooleanSupplier shouldFlick, Runnable onFlick) {
            group.addAction(new PossibleFlickAction(spindexer, shouldFlick, onFlick));
            return this;
        }

        public ParallelBuilder rotateToNext() {
            group.addAction(new RotateAction(spindexer));
            return this;
        }

        public ParallelBuilder rotateTo(BallColor color) {
            group.addAction(new RotateAction(spindexer, color));
            return this;
        }

        public ParallelBuilder rotateCW() {
            group.addAction(new RotateCWAction(spindexer));
            return this;
        }

        public ParallelBuilder rotateCCW() {
            group.addAction(new RotateCCWAction(spindexer));
            return this;
        }

        public ParallelBuilder setDegree(int degrees) {
            group.addAction(new SetDegreeAction(spindexer, degrees));
            return this;
        }

        public ParallelBuilder intake() {
            group.addAction(new IntakeAction(intake));
            return this;
        }

        public ParallelBuilder intake(double seconds) {
            group.addAction(new IntakeAction(intake, seconds));
            return this;
        }

        public ParallelBuilder waitFor(double seconds) {
            group.addAction(new WaitAction(seconds));
            return this;
        }

        public ParallelBuilder run(String name, Runnable runnable) {
            group.addAction(new RunnableAction(name, runnable));
            return this;
        }
    }

    // ==================== Build ====================

    /**
     * Build and return the executor.
     */
    public SpindexerExecutor build() {
        if (!repeatStack.isEmpty()) {
            throw new IllegalStateException("Unclosed repeatWhile() block");
        }
        return executor;
    }

    // ==================== Internal ====================

    private void addAction(SpindexerAction action) {
        if (repeatStack.isEmpty()) {
            executor.addAction(action);
        } else {
            // Add to the current repeat block
            repeatStack.peek().addAction(action);
        }
    }
}
