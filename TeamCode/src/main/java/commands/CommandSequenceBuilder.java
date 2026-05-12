package commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import Constants.EnumConstants.ShootingMode;
import Constants.SpindexerConstants;
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
 *     .parallel(p -> p.delay(.7).limelightScan())
 *     .shoot()
 *     .intakeStart()
 *     .moveTo(path, maxSpeed, false)
 *     .parallel(p -> p.moveTo(returnPath, maxSpeed, false).autoCatalog())
 *     .shoot()
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

    public CommandSequenceBuilder moveTo(Path path) {
        commands.add(new FollowPathCommand(follower, path, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(Path path, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, path, holdEnd));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain) {
        commands.add(new FollowPathCommand(follower, pathChain, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, pathChain, holdEnd));
        return this;
    }

    public CommandSequenceBuilder moveTo(Path path, double maxPower) {
        commands.add(new FollowPathCommand(follower, path, maxPower, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(Path path, double maxPower, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, path, maxPower, holdEnd));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain, double maxPower) {
        commands.add(new FollowPathCommand(follower, pathChain, maxPower, true));
        return this;
    }

    public CommandSequenceBuilder moveTo(PathChain pathChain, double maxPower, boolean holdEnd) {
        commands.add(new FollowPathCommand(follower, pathChain, maxPower, holdEnd));
        return this;
    }

    // ==================== Ball-Collect-MoveTo Methods ====================
    // Follows the path; exits as soon as all 3 distance sensors detect a ball,
    // or shortly after the path completes. The decelStartT/endPower variant tapers
    // speed near the end so the robot doesn't ram into a ball cluster.


    public CommandSequenceBuilder moveToAndCollect(PathChain collectPath, double maxPower) {
        commands.add(new BallCollectMoveToCommand(follower, collectPath, maxPower));
        return this;
    }

    public CommandSequenceBuilder moveToAndCollect(PathChain collectPath, double maxPower,
                                                    double decelStartT, double endPower) {
        commands.add(new BallCollectMoveToCommand(follower, collectPath, maxPower, decelStartT, endPower));
        return this;
    }

    // ==================== Vision-Collect-MoveTo Methods ====================

    /**
     * Vision-guided ball collection with two-stage scan + fallback.
     *
     * Flow:
     *   1. Scan at the current pose. If a corridor is found, drive it.
     *   2. Else rotate in place to secondaryHeadingRad and re-scan.
     *   3. If the re-scan finds a corridor, drive it.
     *   4. Else drive fallbackPath.
     *   5. Return to returnPose while running autoCatalog in parallel.
     *
     * @param detector              the ArtifactDetector
     * @param secondaryHeadingRad   heading to rotate to if first scan is empty
     * @param fallbackPath          path to drive if BOTH scans are empty
     * @param returnPose            pose to return to after collection
     * @param maxPower              maximum drive power
     * @param holdEnd               whether to hold position at the return pose
     * @param delaySec              seconds to wait after collection before returning
     */
    public CommandSequenceBuilder visionCollectWithFallbacksAndCatalog(
            ArtifactDetector detector,
            double secondaryHeadingRad,
            PathChain fallbackPath,
            Pose returnPose,
            double maxPower,
            boolean holdEnd,
            double delaySec) {
        commands.add(new SequentialCommandGroup(
                new VisionCollectWithFallbacksCommand(
                        detector, follower, maxPower,
                        secondaryHeadingRad, fallbackPath),
                new WaitCommand((long)(delaySec * 1000)),
                new ParallelCommandGroup(
                        new DynamicReturnPathCommand(follower, returnPose, maxPower, holdEnd),
                        new AutoCatalogModeCommand(spindexer, intake)
                )
        ));
        return this;
    }

    // ==================== Action Methods ====================

    public CommandSequenceBuilder shoot() {
        commands.add(ShootingCommands.shootThreeBalls(shooter, spindexer));
        return this;
    }

    public CommandSequenceBuilder slowShoot() {
        commands.add(ShootingCommands.slowShootThreeBalls(shooter, spindexer));
        return this;
    }

    public CommandSequenceBuilder setSpindexerMode(ShootingMode mode) {
        commands.add(new InstantCommand(() -> SpindexerConstants.currentMode = mode));
        return this;
    }

    public CommandSequenceBuilder intakeStart() {
        commands.add(new IntakeStartCommand(intake));
        return this;
    }

    public CommandSequenceBuilder intakeStop() {
        commands.add(new IntakeStopCommand(intake));
        return this;
    }

    public CommandSequenceBuilder delay(double seconds) {
        commands.add(new WaitCommand((long)(seconds * 1000)));
        return this;
    }

    // ==================== Parallel Groups ====================

    /**
     * Adds a parallel command group. All commands in the group run simultaneously
     * until all complete.
     *
     * <pre>
     * .parallel(p -> p.moveTo(path, maxSpeed, false).autoCatalog())
     * </pre>
     */
    public CommandSequenceBuilder parallel(Consumer<ParallelBuilder> builder) {
        ParallelBuilder parallelBuilder = new ParallelBuilder(follower, intake, spindexer, limelight);
        builder.accept(parallelBuilder);
        commands.add(parallelBuilder.build());
        return this;
    }

    // ==================== Build ====================

    public Command build() {
        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    // ==================== Parallel Builder Inner Class ====================

    /**
     * Builder for parallel command groups. Provides the subset of fluent methods
     * that make sense to run alongside a drive command.
     */
    public static class ParallelBuilder {
        private final Follower follower;
        private final Intake intake;
        private final Spindexer spindexer;
        private final Limelight limelight;
        private final List<Command> parallelCommands = new ArrayList<>();

        private ParallelBuilder(Follower follower, Intake intake, Spindexer spindexer, Limelight limelight) {
            this.follower = follower;
            this.intake = intake;
            this.spindexer = spindexer;
            this.limelight = limelight;
        }

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

        public ParallelBuilder autoCatalog() {
            parallelCommands.add(new AutoCatalogModeCommand(spindexer, intake));
            return this;
        }

        public ParallelBuilder guaranteeSortedAutoCatalog() {
            parallelCommands.add(new GuaranteeSortedAutoCatalogCommand(spindexer, intake));
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

        ParallelCommandGroup build() {
            return new ParallelCommandGroup(parallelCommands.toArray(new Command[0]));
        }
    }
}
