package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import commands.*;
import Constants.EnumConstants.BallColor;
import Constants.EnumConstants.ShootingMode;
import Constants.SpindexerConstants;
import utility.SpindexerAndMotifStatus;
import pedroPathing.Constants;
import subsystems.Intake;
import subsystems.Limelight;
import subsystems.Shooter;
import subsystems.Spindexer;
import subsystems.Turret;
import vision.ArtifactDetector;


/**
 * Fluent builder API for constructing autonomous command sequences using FTCLib.
 * Provides a declarative way to sequence commands without manual state machine management.
 *
 * Example usage:
 * <pre>
 * autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
 *     .parallel(p -> p.limelightScan().catalog())
 *     .waitForShooterReady(2.0)
 *     .waitForTurretAligned(1.0)
 *     .shoot()
 *     .parallel(p -> p.moveTo(path).intakeStart())
 *     .intakeStop()
 *     .build();
 * </pre>
 */
public class CommandSequenceBuilder {
    protected final Follower follower;
    protected final Intake intake;
    protected final Spindexer spindexer;
    protected final Limelight limelight;
    protected final Shooter shooter;
    protected final Turret turret;

    private final List<Command> commands = new ArrayList<>();

    /**
     * Creates a new command sequence builder.
     *
     * @param follower the path follower
     * @param intake the intake subsystem
     * @param spindexer the spindexer subsystem
     * @param limelight the limelight subsystem
     * @param shooter the shooter subsystem
     * @param turret the turret subsystem
     */
    public CommandSequenceBuilder(Follower follower, Intake intake, Spindexer spindexer,
                                   Limelight limelight, Shooter shooter, Turret turret) {
        this.follower = follower;
        this.intake = intake;
        this.spindexer = spindexer;
        this.limelight = limelight;
        this.shooter = shooter;
        this.turret = turret;
    }

    // ==================== Move-To Path Methods ====================

    /**
     * Adds a move-to-path command.
     *
     * @param path the path to follow
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(Path path) {
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
    }

    /**
     * Adds a move-to-path command with custom hold end setting.
     *
     * @param path the path to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(Path path, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, path, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path-chain command.
     *
     * @param pathChain the path chain to follow
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(PathChain pathChain) {
        commands.add(new FollowPathCommand(follower, pathChain, true));
        return this;
    }

    /**
     * Adds a move-to-path-chain command with custom hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(PathChain pathChain, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, pathChain, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path command with custom speed.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(Path path, double maxPower) {
        commands.add(new FollowPathCommand(follower, path, maxPower, true));
        return this;
    }

    /**
     * Adds a move-to-path command with custom speed and hold end setting.
     *
     * @param path the path to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(Path path, double maxPower, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, path, maxPower, holdEnd));
        return this;
    }

    /**
     * Adds a move-to-path-chain command with custom speed.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(PathChain pathChain, double maxPower) {
        commands.add(new FollowPathCommand(follower, pathChain, maxPower, true));
        return this;
    }

    /**
     * Adds a move-to-path-chain command with custom speed and hold end setting.
     *
     * @param pathChain the path chain to follow
     * @param maxPower the maximum power/speed (0.0-1.0)
     * @param holdEnd whether to hold position at the end
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, pathChain, maxPower, holdEnd));
        return this;
    }

    // ==================== Parametric Speed Methods ====================

    /**
     * Adds a parametric-speed path command. The speed function maps t-value (0.0-1.0)
     * to maxPower (0.0-1.0), allowing variable speed along the path.
     *
     * @param pathChain     the path chain to follow
     * @param speedFunction maps parametric progress to power
     * @param holdEnd       whether to hold position at the end
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveToParametric(PathChain pathChain,
                                                    java.util.function.DoubleUnaryOperator speedFunction,
                                                    boolean holdEnd) {
        commands.add(new ParametricSpeedFollowCommand(follower, pathChain, speedFunction, holdEnd));
        return this;
    }

    // ==================== Ball-Collect-MoveTo Methods ====================
    // Collect phase: follows path until 3 balls or path ends.
    // Then return phase: dynamic straight-line path back to returnPose.
    // AndCatalog variants run autoCatalog in parallel with the return drive.

    public CommandSequenceBuilder ballCollectMoveTo(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd)
        ));
        return this;
    }

    public CommandSequenceBuilder ballCollectMoveTo(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd)
        ));
        return this;
    }

    public CommandSequenceBuilder ballCollectMoveToAndCatalog(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    public CommandSequenceBuilder ballCollectMoveToAndCatalog(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new WaitCommand((long)(delaySec * 1000)),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    public CommandSequenceBuilder ballCollectMoveToAndCatalog(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    public CommandSequenceBuilder ballCollectMoveToAndCatalog(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
        commands.add(new SequentialCommandGroup(
                new BallCollectMoveToCommand(follower, collectPath, maxPower),
                new WaitCommand((long)(delaySec * 1000)),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    // ==================== Vision Pre-Scan ====================

    /**
     * Captures vision frames at the current heading and stores them.
     * Call before rotating to scan heading — the stored frames are merged
     * into the next visionCollectAndCatalog call for better lane coverage.
     * Blocks ~500ms.
     */
    public CommandSequenceBuilder visionPreScan(ArtifactDetector detector) {
        commands.add(new VisionPreScanCommand(detector, follower));
        return this;
    }

    // ==================== Vision-Collect-MoveTo Methods ====================
    // Scans for balls with vision, picks the best lane, follows the chosen path
    // until 3 balls or path end, then returns to returnPose while cataloging.

    /**
     * Vision-guided ball collection cycle.
     * Scans with camera, picks lane, follows chosen path (stops on 3 balls or path end),
     * then returns to returnPose while running autoCatalog in parallel.
     *
     * @param detector   the ArtifactDetector
     * @param goToPaths  array of 3 PathChains indexed by ChosenPath ordinal [PATH_1, PATH_2, PATH_3]
     * @param returnPose the pose to return to after collection (typically shoot pose)
     * @param maxPower   maximum drive power
     * @param holdEnd    whether to hold position at the return pose
     */
    public CommandSequenceBuilder visionCollectAndCatalog(
            ArtifactDetector detector, PathChain[] goToPaths,
            Pose returnPose, double maxPower, boolean holdEnd) {
        commands.add(new SequentialCommandGroup(
                new VisionCollectCommand(detector, follower, goToPaths, maxPower),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    /**
     * Vision-guided ball collection cycle with a delay before return.
     *
     * @param detector   the ArtifactDetector
     * @param goToPaths  array of 3 PathChains indexed by ChosenPath ordinal
     * @param returnPose the pose to return to after collection
     * @param maxPower   maximum drive power
     * @param holdEnd    whether to hold position at the return pose
     * @param delaySec   seconds to wait after collection before returning
     */
    public CommandSequenceBuilder visionCollectAndCatalog(
            ArtifactDetector detector, PathChain[] goToPaths,
            Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
        commands.add(new SequentialCommandGroup(
                new VisionCollectCommand(detector, follower, goToPaths, maxPower),
                new WaitCommand((long)(delaySec * 1000)),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    // ==================== Move-To-Until-Full Path Methods ====================
    // Races FollowPathCommand against WaitForBallsCommand — stops path early when 3 balls detected.

    public CommandSequenceBuilder moveToUntilFull(Path path) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, true), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(Path path, boolean holdEnd) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, holdEnd), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(PathChain pathChain) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, true), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(PathChain pathChain, boolean holdEnd) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, holdEnd), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(Path path, double maxPower) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, maxPower, true), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(Path path, double maxPower, boolean holdEnd) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, maxPower, holdEnd), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(PathChain pathChain, double maxPower) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, maxPower, true), new WaitForBallsCommand(30.0)));
        return this;
    }

    public CommandSequenceBuilder moveToUntilFull(PathChain pathChain, double maxPower, boolean holdEnd) {
        commands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, maxPower, holdEnd), new WaitForBallsCommand(30.0)));
        return this;
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
    public CommandSequenceBuilder moveTo(double x1, double y1, double h1, double x2, double y2, double h2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setLinearHeadingInterpolation(Math.toRadians(h1), Math.toRadians(h2))
                .build();
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
    }

    /**
     * Creates and follows a straight-line path using inline coordinates with tangent heading.
     *
     * @param x1 starting x coordinate (inches)
     * @param y1 starting y coordinate (inches)
     * @param x2 ending x coordinate (inches)
     * @param y2 ending y coordinate (inches)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder moveToTangent(double x1, double y1, double x2, double y2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setTangentHeadingInterpolation()
                .build();
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
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
    public CommandSequenceBuilder moveTo(double x1, double y1, double x2, double y2, double heading) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierLine(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .build();
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
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
    public CommandSequenceBuilder moveToViaCurve(double x1, double y1, double h1, double cx, double cy,
                                                  double x2, double y2, double h2) {
        PathChain path = Constants.pathBuilder(follower)
                .addPath(new com.pedropathing.geometry.BezierCurve(
                        new com.pedropathing.geometry.Pose(x1, y1),
                        new com.pedropathing.geometry.Pose(cx, cy),
                        new com.pedropathing.geometry.Pose(x2, y2)))
                .setLinearHeadingInterpolation(Math.toRadians(h1), Math.toRadians(h2))
                .build();
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
    }

    // ==================== Action Methods ====================

    /**
     * Adds a shooting command that shoots all 3 balls.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder shoot() {
        commands.add(ShootingCommands.shootThreeBalls(shooter, spindexer));
        return this;
    }

    /**
     * Adds a shooting command with specified ball count.
     *
     * @param ballCount number of balls to shoot
     * @return this builder for chaining
     */
    public CommandSequenceBuilder shoot(int ballCount) {
        commands.add(ShootingCommands.shootAllBalls(shooter, spindexer, ballCount));
        return this;
    }

    /**
     * Adds a slow shooting command that shoots all 3 balls with extended flick time (0.30s).
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder slowShoot() {
        commands.add(ShootingCommands.slowShootThreeBalls(shooter, spindexer));
        return this;
    }

    /**
     * Adds a slow shooting command with specified ball count and extended flick time (0.30s).
     *
     * @param ballCount number of balls to shoot
     * @return this builder for chaining
     */
    public CommandSequenceBuilder slowShoot(int ballCount) {
        commands.add(ShootingCommands.slowShootAllBalls(shooter, spindexer, ballCount));
        return this;
    }

    /**
     * Adds a cataloging command.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder catalog() {
        commands.add(new CatalogModeCommand(spindexer, intake));
        return this;
    }

    /**
     * Adds an auto cataloging command that assumes intake is already running.
     * Never stops the intake mid-sequence; only reverses it at the end.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder autoCatalog() {
        commands.add(new AutoCatalogModeCommand(spindexer, intake));
        return this;
    }

    /**
     * Adds a guaranteed sorted auto catalog command.
     * Waits for all sensors to read non-None (up to 0.5s), forces fresh bulk-safe reads,
     * validates colors, then runs sorted auto catalog. Falls back to fast auto if validation fails.
     * Designed to run inside .parallel() groups so sensor-waiting overlaps with driving.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder guaranteeSortedAutoCatalog() {
        commands.add(new GuaranteeSortedAutoCatalogCommand(spindexer, intake));
        return this;
    }

    // ==================== Ramp Scan Methods ====================

    /**
     * Scans the classifier ramp via Limelight pipeline 6 (Python SnapScript).
     * Counts non-zero entries in getPythonOutput() and updates RampTracker.
     * Falls back to manual counter if scan times out.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder rampScan() {
        commands.add(new RampScanCommand(limelight));
        return this;
    }

    /**
     * Scans the classifier ramp with a custom timeout.
     *
     * @param timeout timeout in seconds
     * @return this builder for chaining
     */
    public CommandSequenceBuilder rampScan(double timeout) {
        commands.add(new RampScanCommand(limelight, timeout));
        return this;
    }

    /**
     * Shifts MotifPattern by rampCount % 3 so the sorted catalog uses the correct
     * shooting order. Saves the original motif on first call (from limelightScan).
     * Call after rampScan() and before guaranteeSortedAutoCatalog().
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder shiftMotif() {
        commands.add(new InstantCommand(() -> SpindexerAndMotifStatus.RampTracker.shiftMotifForRamp()));
        return this;
    }

    /**
     * Resets the ramp ball counter to 0.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder rampClear() {
        commands.add(new InstantCommand(() -> SpindexerAndMotifStatus.RampTracker.clear()));
        return this;
    }

    /**
     * Sets the initial spindexer ball pattern using the default preload.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder preload() {
        commands.add(new PreloadCommand());
        return this;
    }

    /**
     * Sets the initial spindexer ball pattern with a custom configuration.
     *
     * @param pattern array of 3 BallColors: [Intake, Shooter, TopStorage]
     * @return this builder for chaining
     */
    public CommandSequenceBuilder preload(BallColor[] pattern) {
        commands.add(new PreloadCommand(pattern));
        return this;
    }

    /**
     * Sets the initial spindexer ball pattern with explicit ball colors.
     *
     * @param intake ball color in intake slot (slot 0)
     * @param shooter ball color in shooter slot (slot 1)
     * @param topStorage ball color in top storage slot (slot 2)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder preload(BallColor intake, BallColor shooter, BallColor topStorage) {
        commands.add(new PreloadCommand(intake, shooter, topStorage));
        return this;
    }

    /**
     * Sets the spindexer shooting mode.
     * Use this to switch between Fast and Sorted modes mid-sequence.
     *
     * @param mode the shooting mode (Fast or Sorted)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder setSpindexerMode(ShootingMode mode) {
        commands.add(new InstantCommand(() -> SpindexerConstants.currentMode = mode));
        return this;
    }

    /**
     * Enables or disables shoot-while-moving lead compensation.
     *
     * @param enabled true to enable, false to disable
     * @return this builder for chaining
     */
    public CommandSequenceBuilder setShootWhileMoving(boolean enabled) {
        commands.add(new SetShootWhileMovingCommand(enabled));
        return this;
    }

    /**
     * Adds an intake start command.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder intakeStart() {
        commands.add(new IntakeStartCommand(intake));
        return this;
    }

    /**
     * Adds an intake stop command.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder intakeStop() {
        commands.add(new IntakeStopCommand(intake));
        return this;
    }

    /**
     * Adds a limelight scan command.
     *
     * @return this builder for chaining
     */
    public CommandSequenceBuilder limelightScan() {
        commands.add(new LimelightScanCommand(limelight));
        return this;
    }

    /**
     * Adds a limelight scan command with custom timeout.
     *
     * @param timeout timeout in seconds
     * @return this builder for chaining
     */
    public CommandSequenceBuilder limelightScan(double timeout) {
        commands.add(new LimelightScanCommand(limelight, timeout));
        return this;
    }

    /**
     * Adds a delay command.
     *
     * @param seconds the delay duration in seconds
     * @return this builder for chaining
     */
    public CommandSequenceBuilder delay(double seconds) {
        commands.add(new WaitCommand((long)(seconds * 1000)));
        return this;
    }

    /**
     * Waits for shooter to reach target velocity.
     *
     * @param timeout max seconds to wait
     * @return this builder for chaining
     */
    public CommandSequenceBuilder waitForShooterReady(double timeout) {
        commands.add(new WaitForShooterReadyCommand(shooter, timeout));
        return this;
    }

    /**
     * Waits for turret to be aligned (not out of range).
     *
     * @param timeout max seconds to wait
     * @return this builder for chaining
     */
    public CommandSequenceBuilder waitForTurretAligned(double timeout) {
        commands.add(new WaitForTurretAlignedCommand(turret, timeout));
        return this;
    }

    // ==================== Turret Pre-Aim Methods ====================

    /**
     * Sets the turret to a specific angle in degrees.
     * Useful for manually positioning the turret.
     *
     * @param degrees the target angle in turret degrees
     * @return this builder for chaining
     */
    public CommandSequenceBuilder setTurretAngle(double degrees) {
        commands.add(new SetTurretAngleCommand(turret, degrees));
        return this;
    }

    /**
     * Pre-aims the turret to the goal as if the robot were at the specified position.
     * Useful for aiming the turret before arriving at a shooting position.
     *
     * @param robotX hypothetical robot X position (inches)
     * @param robotY hypothetical robot Y position (inches)
     * @param robotHeadingDeg hypothetical robot heading (degrees)
     * @return this builder for chaining
     */
    public CommandSequenceBuilder preAimTurret(double robotX, double robotY, double robotHeadingDeg) {
        commands.add(new SetTurretAngleCommand(turret, robotX, robotY, robotHeadingDeg));
        return this;
    }

    /**
     * Pre-aims the turret to the goal based on the end position of a path chain.
     * Calculates where the robot will be at the end of the path and aims accordingly.
     *
     * @param pathChain the path chain to get the end position from
     * @return this builder for chaining
     */
    public CommandSequenceBuilder preAimToPathEnd(PathChain pathChain) {
        Pose endPose = pathChain.getPath(pathChain.size() - 1).getLastControlPoint();
        commands.add(new SetTurretAngleCommand(turret, endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading())));
        return this;
    }

    /**
     * Adds a custom command directly.
     *
     * @param command the command to add
     * @return this builder for chaining
     */
    public CommandSequenceBuilder addCommand(Command command) {
        commands.add(command);
        return this;
    }

    // ==================== Parallel Groups ====================

    /**
     * Adds a parallel command group.
     * All commands in the group will run simultaneously until all complete.
     *
     * Example:
     * <pre>
     * .parallel(p -> p.moveTo(path).catalog())
     * </pre>
     *
     * @param builder a consumer that adds commands to the parallel group
     * @return this builder for chaining
     */
    public CommandSequenceBuilder parallel(Consumer<ParallelBuilder> builder) {
        ParallelBuilder parallelBuilder = new ParallelBuilder(follower, intake,
                spindexer, limelight, shooter, turret);
        builder.accept(parallelBuilder);
        commands.add(parallelBuilder.build());
        return this;
    }

    // ==================== Build ====================

    /**
     * Builds and returns the configured command sequence.
     * Call this at the end of your sequence definition.
     *
     * @return the configured SequentialCommandGroup
     */
    public Command build() {
        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    // ==================== Parallel Builder Inner Class ====================

    /**
     * Builder for parallel command groups.
     * Provides the same fluent methods but adds to a parallel group instead.
     */
    public static class ParallelBuilder {
        private final Follower follower;
        private final Intake intake;
        private final Spindexer spindexer;
        private final Limelight limelight;
        private final Shooter shooter;
        private final Turret turret;
        private final List<Command> parallelCommands = new ArrayList<>();

        private ParallelBuilder(Follower follower, Intake intake, Spindexer spindexer,
                                Limelight limelight, Shooter shooter, Turret turret) {
            this.follower = follower;
            this.intake = intake;
            this.spindexer = spindexer;
            this.limelight = limelight;
            this.shooter = shooter;
            this.turret = turret;
        }

        // Path methods
        public ParallelBuilder moveTo(Path path) {
            parallelCommands.add(new FollowPathCommand(follower, path, true));
            return this;
        }

        public ParallelBuilder moveTo(Path path, double maxPower) {
            parallelCommands.add(new FollowPathCommand(follower, path, maxPower, true));
            return this;
        }

        public ParallelBuilder moveTo(Path path, boolean holdEnd) {
            parallelCommands.add(new FollowPathCommand(follower, path, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(Path path, double maxPower, boolean holdEnd) {
            parallelCommands.add(new FollowPathCommand(follower, path, maxPower, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain) {
            parallelCommands.add(new FollowPathCommand(follower, pathChain, true));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, double maxPower) {
            parallelCommands.add(new FollowPathCommand(follower, pathChain, maxPower, true));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, boolean holdEnd) {
            parallelCommands.add(new FollowPathCommand(follower, pathChain, holdEnd));
            return this;
        }

        public ParallelBuilder moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
            parallelCommands.add(new FollowPathCommand(follower, pathChain, maxPower, holdEnd));
            return this;
        }

        // Ball-collect-moveTo methods (collect then auto-return)
        public ParallelBuilder ballCollectMoveTo(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd)
            ));
            return this;
        }

        public ParallelBuilder ballCollectMoveTo(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd)
            ));
            return this;
        }

        public ParallelBuilder ballCollectMoveToAndCatalog(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        public ParallelBuilder ballCollectMoveToAndCatalog(Path collectPath, Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new WaitCommand((long)(delaySec * 1000)),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        public ParallelBuilder ballCollectMoveToAndCatalog(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        public ParallelBuilder ballCollectMoveToAndCatalog(PathChain collectPath, Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
            parallelCommands.add(new SequentialCommandGroup(
                    new BallCollectMoveToCommand(follower, collectPath, maxPower),
                    new WaitCommand((long)(delaySec * 1000)),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        // Vision-collect methods
        public ParallelBuilder visionCollectAndCatalog(
                ArtifactDetector detector, PathChain[] goToPaths,
                Pose returnPose, double maxPower, boolean holdEnd) {
            parallelCommands.add(new SequentialCommandGroup(
                    new VisionCollectCommand(detector, follower, goToPaths, maxPower),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        public ParallelBuilder visionCollectAndCatalog(
                ArtifactDetector detector, PathChain[] goToPaths,
                Pose returnPose, double maxPower, boolean holdEnd, double delaySec) {
            parallelCommands.add(new SequentialCommandGroup(
                    new VisionCollectCommand(detector, follower, goToPaths, maxPower),
                    new WaitCommand((long)(delaySec * 1000)),
                    new ParallelCommandGroup(
                            new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                            new AutoCatalogModeCommand(spindexer, intake)
                    )
            ));
            return this;
        }

        // Move-to-until-full methods (race path against ball detection)
        public ParallelBuilder moveToUntilFull(Path path) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, true), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(Path path, boolean holdEnd) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, holdEnd), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(PathChain pathChain) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, true), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(PathChain pathChain, boolean holdEnd) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, holdEnd), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(Path path, double maxPower) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, maxPower, true), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(Path path, double maxPower, boolean holdEnd) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, path, maxPower, holdEnd), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(PathChain pathChain, double maxPower) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, maxPower, true), new WaitForBallsCommand(30.0)));
            return this;
        }

        public ParallelBuilder moveToUntilFull(PathChain pathChain, double maxPower, boolean holdEnd) {
            parallelCommands.add(new ParallelRaceGroup(new FollowPathCommand(follower, pathChain, maxPower, holdEnd), new WaitForBallsCommand(30.0)));
            return this;
        }

        // Action methods
        public ParallelBuilder shoot() {
            parallelCommands.add(ShootingCommands.shootThreeBalls(shooter, spindexer));
            return this;
        }

        public ParallelBuilder shootAfterDelay(double seconds) {
            parallelCommands.add(new SequentialCommandGroup(
                    new WaitCommand((long)(seconds * 1000)),
                    ShootingCommands.shootThreeBalls(shooter, spindexer)
            ));
            return this;
        }

        public ParallelBuilder shoot(int ballCount) {
            parallelCommands.add(ShootingCommands.shootAllBalls(shooter, spindexer, ballCount));
            return this;
        }

        public ParallelBuilder slowShoot() {
            parallelCommands.add(ShootingCommands.slowShootThreeBalls(shooter, spindexer));
            return this;
        }

        public ParallelBuilder slowShoot(int ballCount) {
            parallelCommands.add(ShootingCommands.slowShootAllBalls(shooter, spindexer, ballCount));
            return this;
        }

        public ParallelBuilder catalog() {
            parallelCommands.add(new CatalogModeCommand(spindexer, intake));
            return this;
        }

        public ParallelBuilder autoCatalog() {
            parallelCommands.add(new AutoCatalogModeCommand(spindexer, intake));
            return this;
        }

        public ParallelBuilder guaranteeSortedAutoCatalog() {
            parallelCommands.add(new GuaranteeSortedAutoCatalogCommand(spindexer, intake));
            return this;
        }

        public ParallelBuilder rampScan() {
            parallelCommands.add(new RampScanCommand(limelight));
            return this;
        }

        public ParallelBuilder rampScan(double timeout) {
            parallelCommands.add(new RampScanCommand(limelight, timeout));
            return this;
        }

        public ParallelBuilder rampClear() {
            parallelCommands.add(new InstantCommand(() -> SpindexerAndMotifStatus.RampTracker.clear()));
            return this;
        }

        public ParallelBuilder shiftMotif() {
            parallelCommands.add(new InstantCommand(() -> SpindexerAndMotifStatus.RampTracker.shiftMotifForRamp()));
            return this;
        }

        public ParallelBuilder preload() {
            parallelCommands.add(new PreloadCommand());
            return this;
        }

        public ParallelBuilder intakeStart() {
            parallelCommands.add(new IntakeStartCommand(intake));
            return this;
        }

        public ParallelBuilder intakeStop() {
            parallelCommands.add(new IntakeStopCommand(intake));
            return this;
        }

        public ParallelBuilder setSpindexerMode(ShootingMode mode) {
            parallelCommands.add(new InstantCommand(() -> SpindexerConstants.currentMode = mode));
            return this;
        }

        public ParallelBuilder setShootWhileMoving(boolean enabled) {
            parallelCommands.add(new SetShootWhileMovingCommand(enabled));
            return this;
        }

        public ParallelBuilder limelightScan() {
            parallelCommands.add(new LimelightScanCommand(limelight));
            return this;
        }

        public ParallelBuilder delay(double seconds) {
            parallelCommands.add(new WaitCommand((long)(seconds * 1000)));
            return this;
        }

        public ParallelBuilder waitForShooterReady(double timeout) {
            parallelCommands.add(new WaitForShooterReadyCommand(shooter, timeout));
            return this;
        }

        public ParallelBuilder waitForTurretAligned(double timeout) {
            parallelCommands.add(new WaitForTurretAlignedCommand(turret, timeout));
            return this;
        }

        public ParallelBuilder setTurretAngle(double degrees) {
            parallelCommands.add(new SetTurretAngleCommand(turret, degrees));
            return this;
        }

        public ParallelBuilder preAimTurret(double robotX, double robotY, double robotHeadingDeg) {
            parallelCommands.add(new SetTurretAngleCommand(turret, robotX, robotY, robotHeadingDeg));
            return this;
        }

        public ParallelBuilder preAimToPathEnd(PathChain pathChain) {
            Pose endPose = pathChain.getPath(pathChain.size() - 1).getLastControlPoint();
            parallelCommands.add(new SetTurretAngleCommand(turret, endPose.getX(), endPose.getY(), Math.toDegrees(endPose.getHeading())));
            return this;
        }

        public ParallelBuilder visionPreScan(ArtifactDetector detector) {
            parallelCommands.add(new VisionPreScanCommand(detector, follower));
            return this;
        }

        public ParallelBuilder addCommand(Command command) {
            parallelCommands.add(command);
            return this;
        }

        /**
         * Builds the parallel command group.
         */
        ParallelCommandGroup build() {
            return new ParallelCommandGroup(parallelCommands.toArray(new Command[0]));
        }
    }
}
