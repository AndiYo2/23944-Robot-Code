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

/**
 * Abstract base class for action builders.
 * Contains all shared action creation methods used by both AutonSequence and ParallelBuilder.
 *
 * @param <T> the concrete builder type (for fluent API chaining)
 */
public abstract class ActionBuilderBase<T extends ActionBuilderBase<T>> {
    protected final Follower follower;
    protected final Intake intake;
    protected final CatalogManager catalogManager;
    protected final ShootingSequenceManager sequenceManager;
    protected final Limelight limelight;

    protected ActionBuilderBase(Follower follower, Intake intake, CatalogManager catalogManager,
                                ShootingSequenceManager sequenceManager, Limelight limelight) {
        this.follower = follower;
        this.intake = intake;
        this.catalogManager = catalogManager;
        this.sequenceManager = sequenceManager;
        this.limelight = limelight;
    }

    /**
     * Adds an action to the underlying container (executor or group).
     * Implemented by subclasses to add to their specific target.
     */
    protected abstract void addActionInternal(Action action);

    /**
     * Returns this builder for fluent API chaining.
     * Implemented by subclasses to return the correct type.
     */
    protected abstract T self();

    // ==================== Move-To Path Methods ====================

    /**
     * Adds a move-to-path action.
     *
     * @param path the path to follow
     * @return this builder for chaining
     */
    public T moveTo(Path path) {
        addActionInternal(new MoveToAction(follower, path, true));
        return self();
    }

    /**
     * Adds a move-to-path action with custom hold end setting.
     *
     * @param path the path to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public T moveTo(Path path, boolean holdEnd) {
        addActionInternal(new MoveToAction(follower, path, holdEnd));
        return self();
    }

    /**
     * Adds a move-to-path-chain action.
     *
     * @param pathChain the path chain to follow
     * @return this builder for chaining
     */
    public T moveTo(PathChain pathChain) {
        addActionInternal(new MoveToAction(follower, pathChain, true));
        return self();
    }

    /**
     * Adds a move-to-path-chain action with custom hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public T moveTo(PathChain pathChain, boolean holdEnd) {
        addActionInternal(new MoveToAction(follower, pathChain, holdEnd));
        return self();
    }

    /**
     * Adds a move-to-path action with custom speed.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0, where 0.5 = 50% speed)
     * @return this builder for chaining
     */
    public T moveTo(Path path, double maxPower) {
        addActionInternal(new MoveToAction(follower, path, maxPower, true));
        return self();
    }

    /**
     * Adds a move-to-path action with custom speed and hold end setting.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public T moveTo(Path path, double maxPower, boolean holdEnd) {
        addActionInternal(new MoveToAction(follower, path, maxPower, holdEnd));
        return self();
    }

    /**
     * Adds a move-to-path-chain action with custom speed.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @return this builder for chaining
     */
    public T moveTo(PathChain pathChain, double maxPower) {
        addActionInternal(new MoveToAction(follower, pathChain, maxPower, true));
        return self();
    }

    /**
     * Adds a move-to-path-chain action with custom speed and hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public T moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
        addActionInternal(new MoveToAction(follower, pathChain, maxPower, holdEnd));
        return self();
    }

    // ==================== Inline Coordinate Path Methods ====================

    /**
     * Creates and follows a straight-line path using inline coordinates with linear heading interpolation.
     *
     * @param x1 starting x coordinate (inches)
     * @param y1 starting y coordinate (inches)
     * @param h1 starting heading (degrees)
     * @param x2 ending x coordinate (inches)
     * @param y2 ending y coordinate (inches)
     * @param h2 ending heading (degrees)
     * @return this builder for chaining
     */
    public T moveTo(double x1, double y1, double h1, double x2, double y2, double h2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setLinearHeadingInterpolation(Math.toRadians(h1), Math.toRadians(h2))
                .build();
        addActionInternal(new MoveToAction(follower, path, true));
        return self();
    }

    /**
     * Creates and follows a straight-line path using inline coordinates with tangent heading.
     * The robot heading will automatically align with the path direction.
     *
     * @param x1 starting x coordinate (inches)
     * @param y1 starting y coordinate (inches)
     * @param x2 ending x coordinate (inches)
     * @param y2 ending y coordinate (inches)
     * @return this builder for chaining
     */
    public T moveToTangent(double x1, double y1, double x2, double y2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setTangentHeadingInterpolation()
                .build();
        addActionInternal(new MoveToAction(follower, path, true));
        return self();
    }

    /**
     * Creates and follows a straight-line path using inline coordinates with constant heading.
     *
     * @param x1 starting x coordinate (inches)
     * @param y1 starting y coordinate (inches)
     * @param x2 ending x coordinate (inches)
     * @param y2 ending y coordinate (inches)
     * @param heading constant heading to maintain (degrees)
     * @return this builder for chaining
     */
    public T moveTo(double x1, double y1, double x2, double y2, double heading) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .build();
        addActionInternal(new MoveToAction(follower, path, true));
        return self();
    }

    /**
     * Creates and follows a curved Bezier path using inline coordinates with linear heading interpolation.
     *
     * @param x1 starting x coordinate (inches)
     * @param y1 starting y coordinate (inches)
     * @param h1 starting heading (degrees)
     * @param cx control point x coordinate (inches)
     * @param cy control point y coordinate (inches)
     * @param x2 ending x coordinate (inches)
     * @param y2 ending y coordinate (inches)
     * @param h2 ending heading (degrees)
     * @return this builder for chaining
     */
    public T moveToViaCurve(double x1, double y1, double h1, double cx, double cy,
                            double x2, double y2, double h2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierCurve(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(cx, cy),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setLinearHeadingInterpolation(Math.toRadians(h1), Math.toRadians(h2))
                .build();
        addActionInternal(new MoveToAction(follower, path, true));
        return self();
    }

    // ==================== Action Methods ====================

    /**
     * Adds a shooting action.
     *
     * @return this builder for chaining
     */
    public T shoot() {
        addActionInternal(new ShootAction(sequenceManager));
        return self();
    }

    /**
     * Adds a cataloging action.
     *
     * @return this builder for chaining
     */
    public T catalog() {
        addActionInternal(new CatalogAction(catalogManager));
        return self();
    }

    /**
     * Sets the initial spindexer ball pattern using the default preload (Purple, Purple, Green).
     * Use this instead of catalog() when ball positions are known at match start.
     *
     * @return this builder for chaining
     */
    public T preload() {
        addActionInternal(new PreloadAction());
        return self();
    }

    /**
     * Sets the initial spindexer ball pattern with a custom configuration.
     *
     * @param pattern array of 3 BallColors: [Intake, Shooter, TopStorage]
     * @return this builder for chaining
     */
    public T preload(EnumConstants.BallColor[] pattern) {
        addActionInternal(new PreloadAction(pattern));
        return self();
    }

    /**
     * Sets the initial spindexer ball pattern with explicit ball colors.
     *
     * @param intake ball color in intake slot (slot 0)
     * @param shooter ball color in shooter slot (slot 1)
     * @param topStorage ball color in top storage slot (slot 2)
     * @return this builder for chaining
     */
    public T preload(EnumConstants.BallColor intake,
                     EnumConstants.BallColor shooter,
                     EnumConstants.BallColor topStorage) {
        addActionInternal(new PreloadAction(intake, shooter, topStorage));
        return self();
    }

    /**
     * Adds an intake start action.
     *
     * @return this builder for chaining
     */
    public T intakeStart() {
        addActionInternal(new IntakeControlAction(intake, true));
        return self();
    }

    /**
     * Adds an intake stop action.
     *
     * @return this builder for chaining
     */
    public T intakeStop() {
        addActionInternal(new IntakeControlAction(intake, false));
        return self();
    }

    /**
     * Adds a limelight scan action.
     *
     * @return this builder for chaining
     */
    public T limelightScan() {
        addActionInternal(new LimelightScanAction(limelight));
        return self();
    }

    /**
     * Adds a delay action.
     *
     * @param seconds the delay duration in seconds
     * @return this builder for chaining
     */
    public T delay(double seconds) {
        addActionInternal(new DelayAction(seconds));
        return self();
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
    public T addAction(Action action) {
        addActionInternal(action);
        return self();
    }
}
