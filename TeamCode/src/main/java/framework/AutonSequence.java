package framework;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import framework.actions.*;
import pedroPathing.Constants;
import subsystems.Intake;
import subsystems.Limelight;
import utility.CatalogManager;
import Constants.EnumConstants;
import utility.ShootingSequenceManager;

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
public class AutonSequence extends ActionBuilderBase<AutonSequence> {
    private final ActionExecutor executor;

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
        super(follower, intake, catalogManager, sequenceManager, limelight);
        this.executor = new ActionExecutor();
    }

    @Override
    protected void addActionInternal(Action action) {
        executor.addAction(action);
    }

    @Override
    protected AutonSequence self() {
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
        ParallelBuilder parallelBuilder = new ParallelBuilder(group, follower, intake,
                catalogManager, sequenceManager, limelight);
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
    public static class ParallelBuilder extends ActionBuilderBase<ParallelBuilder> {
        private final ActionGroup group;

        private ParallelBuilder(ActionGroup group, Follower follower, Intake intake,
                               CatalogManager catalogManager, ShootingSequenceManager sequenceManager,
                               Limelight limelight) {
            super(follower, intake, catalogManager, sequenceManager, limelight);
            this.group = group;
        }

        @Override
        protected void addActionInternal(Action action) {
            group.addAction(action);
        }

        @Override
        protected ParallelBuilder self() {
            return this;
        }
    }
}
