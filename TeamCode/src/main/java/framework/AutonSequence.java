package framework;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import framework.actions.*;
import subsystems.Intake;
import subsystems.Limelight;
import utility.CatalogManager;
import utility.Shooting.ShootingSequenceManager;

import java.util.function.Consumer;

/**
 * Fluent builder API for constructing autonomous routines.
 * Provides a declarative way to sequence actions without manual state machine management.
 *
 * Example usage:
 * <pre>
 * executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
 *     .parallel(p -> p.limelightScan().catalog())
 *     .shoot()
 *     .parallel(p -> p.moveTo(path).intakeStart())
 *     .intakeStop()
 *     .build();
 * </pre>
 */
public class AutonSequence {
    private final ActionExecutor executor;
    private final Follower follower;
    private final Intake intake;
    private final CatalogManager catalogManager;
    private final ShootingSequenceManager sequenceManager;
    private final Limelight limelight;

    /**
     * Creates a new autonomous sequence builder.
     *
     * @param follower the path follower
     * @param intake the intake subsystem
     * @param catalogManager the catalog manager
     * @param sequenceManager the shooting sequence manager
     * @param limelight the limelight subsystem
     */
    public AutonSequence(Follower follower, Intake intake, CatalogManager catalogManager,
                         ShootingSequenceManager sequenceManager, Limelight limelight) {
        this.executor = new ActionExecutor();
        this.follower = follower;
        this.intake = intake;
        this.catalogManager = catalogManager;
        this.sequenceManager = sequenceManager;
        this.limelight = limelight;
    }

    /**
     * Adds a move-to-path action.
     *
     * @param path the path to follow
     * @return this builder for chaining
     */
    public AutonSequence moveTo(Path path) {
        executor.addAction(new MoveToAction(follower, path, true));
        return this;
    }

    /**
     * Adds a move-to-path action with custom hold end setting.
     *
     * @param path the path to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public AutonSequence moveTo(Path path, boolean holdEnd) {
        executor.addAction(new MoveToAction(follower, path, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path-chain action.
     *
     * @param pathChain the path chain to follow
     * @return this builder for chaining
     */
    public AutonSequence moveTo(PathChain pathChain) {
        executor.addAction(new MoveToAction(follower, pathChain, true));
        return this;
    }

    /**
     * Adds a move-to-path-chain action with custom hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public AutonSequence moveTo(PathChain pathChain, boolean holdEnd) {
        executor.addAction(new MoveToAction(follower, pathChain, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path action with custom speed.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0, where 0.5 = 50% speed)
     * @return this builder for chaining
     */
    public AutonSequence moveTo(Path path, double maxPower) {
        executor.addAction(new MoveToAction(follower, path, maxPower, true));
        return this;
    }

    /**
     * Adds a move-to-path action with custom speed and hold end setting.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public AutonSequence moveTo(Path path, double maxPower, boolean holdEnd) {
        executor.addAction(new MoveToAction(follower, path, maxPower, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path-chain action with custom speed.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @return this builder for chaining
     */
    public AutonSequence moveTo(PathChain pathChain, double maxPower) {
        executor.addAction(new MoveToAction(follower, pathChain, maxPower, true));
        return this;
    }

    /**
     * Adds a move-to-path-chain action with custom speed and hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public AutonSequence moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
        executor.addAction(new MoveToAction(follower, pathChain, maxPower, holdEnd));
        return this;
    }

    /**
     * Adds a shooting action.
     *
     * @return this builder for chaining
     */
    public AutonSequence shoot() {
        executor.addAction(new ShootAction(sequenceManager));
        return this;
    }

    /**
     * Adds a cataloging action.
     *
     * @return this builder for chaining
     */
    public AutonSequence catalog() {
        executor.addAction(new CatalogAction(catalogManager));
        return this;
    }

    /**
     * Adds an intake start action.
     *
     * @return this builder for chaining
     */
    public AutonSequence intakeStart() {
        executor.addAction(new IntakeControlAction(intake, true));
        return this;
    }

    /**
     * Adds an intake stop action.
     *
     * @return this builder for chaining
     */
    public AutonSequence intakeStop() {
        executor.addAction(new IntakeControlAction(intake, false));
        return this;
    }

    /**
     * Adds a limelight scan action.
     *
     * @return this builder for chaining
     */
    public AutonSequence limelightScan() {
        executor.addAction(new LimelightScanAction(limelight));
        return this;
    }

    /**
     * Adds a delay action.
     *
     * @param seconds the delay duration in seconds
     * @return this builder for chaining
     */
    public AutonSequence delay(double seconds) {
        executor.addAction(new DelayAction(seconds));
        return this;
    }

    /**
     * Adds a custom action directly.
     * Use this for custom actions that don't have a dedicated fluent method.
     *
     * Example:
     * <pre>
     * .addAction(new CustomAction(...))
     * </pre>
     *
     * @param action the action to add
     * @return this builder for chaining
     */
    public AutonSequence addAction(Action action) {
        executor.addAction(action);
        return this;
    }

    /**
     * Adds a parallel action group.
     * All actions in the group will run simultaneously until all complete.
     *
     * Example:
     * <pre>
     * .parallel(p -> p.moveTo(path).catalog())
     * </pre>
     *
     * @param builder a consumer that adds actions to the parallel group
     * @return this builder for chaining
     */
    public AutonSequence parallel(Consumer<ParallelBuilder> builder) {
        ActionGroup group = new ActionGroup(ActionGroup.ExecutionMode.PARALLEL);
        ParallelBuilder parallelBuilder = new ParallelBuilder(group);
        builder.accept(parallelBuilder);
        executor.addAction(group);
        return this;
    }

    /**
     * Builds and returns the configured ActionExecutor.
     * Call this at the end of your sequence definition.
     *
     * @return the configured executor
     */
    public ActionExecutor build() {
        return executor;
    }

    /**
     * Builder for parallel action groups.
     * Provides the same fluent methods but adds to a parallel group instead.
     */
    public class ParallelBuilder {
        private final ActionGroup group;

        private ParallelBuilder(ActionGroup group) {
            this.group = group;
        }

        public ParallelBuilder moveTo(Path path) {
            group.addAction(new MoveToAction(follower, path, true));
            return this;
        }

        public ParallelBuilder moveTo(Path path, boolean holdEnd) {
            group.addAction(new MoveToAction(follower, path, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain) {
            group.addAction(new MoveToAction(follower, pathChain, true));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, boolean holdEnd) {
            group.addAction(new MoveToAction(follower, pathChain, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(Path path, double maxPower) {
            group.addAction(new MoveToAction(follower, path, maxPower, true));
            return this;
        }

        public ParallelBuilder moveTo(Path path, double maxPower, boolean holdEnd) {
            group.addAction(new MoveToAction(follower, path, maxPower, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, double maxPower) {
            group.addAction(new MoveToAction(follower, pathChain, maxPower, true));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
            group.addAction(new MoveToAction(follower, pathChain, maxPower, holdEnd));
            return this;
        }

        public ParallelBuilder shoot() {
            group.addAction(new ShootAction(sequenceManager));
            return this;
        }

        public ParallelBuilder catalog() {
            group.addAction(new CatalogAction(catalogManager));
            return this;
        }

        public ParallelBuilder intakeStart() {
            group.addAction(new IntakeControlAction(intake, true));
            return this;
        }

        public ParallelBuilder intakeStop() {
            group.addAction(new IntakeControlAction(intake, false));
            return this;
        }

        public ParallelBuilder limelightScan() {
            group.addAction(new LimelightScanAction(limelight));
            return this;
        }

        public ParallelBuilder delay(double seconds) {
            group.addAction(new DelayAction(seconds));
            return this;
        }

        public ParallelBuilder addAction(Action action) {
            group.addAction(action);
            return this;
        }
    }
}
