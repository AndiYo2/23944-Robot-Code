package Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import framework.AutonSequence;

/**
 * Example autonomous demonstrating streamlined path creation approaches:
 * 1. Visualizer Paths class - paste exported code from Pedro Pathing Visualizer
 * 2. Inline coordinates - quick paths without naming
 * 3. Hybrid usage - mix both approaches as needed
 */
@Autonomous(name = "BlueBackAutonCorner (Quick)", group = "Autonomous")
public class BlueBackAutonCornerQuick extends AutonTemplate {
    // Using visualizer export approach - paste the Paths class directly
    private Paths paths;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(new Pose(56.5, 8.5, Math.toRadians(90)));

        // Instantiate paths from visualizer export
        paths = new Paths(follower);
    }

    @Override
    protected void autonomousPathUpdate() {
        // Not used - framework handles execution
    }

    @Override
    public void init() {
        super.init();

        // Build the autonomous sequence using hybrid approach
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)
                // Initial: scan and catalog preloaded balls, then shoot
                .parallel(p -> p.limelightScan().catalog())
                .shoot()

                // Move to first pickup with intake (using visualizer path)
                .parallel(p -> p.moveTo(paths.shootToFirst).intakeStart())
                .intakeStop()

                // Catalog while moving to first shoot pose (using visualizer path), then shoot
                .parallel(p -> p.moveTo(paths.firstToShoot).catalog())
                .shoot()

                // Move through second pickup sequence (using inline coordinates for simple movements)
                // This demonstrates inline coordinate usage - no need to define poses or paths!
                .moveToTangent(56.5, 15.5, 13, 49.5)    // firstShoot -> secondPickup1
                .moveToTangent(13, 49.5, 11.25, 22.5)   // secondPickup1 -> secondPickup2
                .parallel(p -> p.moveToTangent(11.25, 22.5, 11, 9.5).intakeStart())  // secondPickup2 -> secondPickup3
                .intakeStop()

                // Back to shoot (using visualizer path for curved path)
                .parallel(p -> p.moveTo(paths.secondToShoot).catalog())
                .shoot()

                // Park using inline coordinates
                .moveToTangent(54.5, 13, 54.5, 40)
                .build();
    }

    /**
     * Paths exported from Pedro Pathing Visualizer.
     *
     * USAGE:
     * 1. Design your paths in the visualizer at https://visualizer.pedropathing.com/
     * 2. Export the code (rename Path1, Path2, etc. to semantic names in the visualizer)
     * 3. Paste the class here
     * 4. Instantiate in buildPaths(): paths = new VisualizerPaths(follower);
     * 5. Use in sequence: .moveTo(paths.shootToFirst)
     *
     * This example shows manually created paths following the visualizer export pattern.
     */
    private static class Paths {
        public PathChain shootToFirst;
        public PathChain firstToShoot;
        public PathChain secondToShoot;

        public Paths(Follower follower) {
            // First path: curved path from start to first pickup
            shootToFirst = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(56.5, 8.5),
                            new Pose(56, 31),  // control point
                            new Pose(23, 37.5)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            // Second path: straight line from first pickup to shoot position
            firstToShoot = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(23, 37.5),
                            new Pose(56.5, 15.5)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(135))
                    .build();

            // Third path: curved path from third pickup back to shoot
            secondToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(11, 9.5),
                            new Pose(29.5, 20),  // control point
                            new Pose(54.5, 13)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(90))
                    .build();
        }
    }
}
