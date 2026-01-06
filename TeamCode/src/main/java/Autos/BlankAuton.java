package Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import framework.AutonSequence;

/**
 * Blank autonomous template - clone this to create new autos quickly!
 *
 * USAGE:
 * 1. Copy this file and rename (e.g., RedFrontAuto.java)
 * 2. Update the class name and @Autonomous annotation
 * 3. Set your starting pose
 * 4. Add paths using one of three approaches:
 *    - Inline coordinates: .moveTo(x1, y1, h1, x2, y2, h2)
 *    - Visualizer paste: Create VisualizerPaths class and instantiate
 *    - Named paths: Define Path/PathChain fields in buildPaths()
 * 5. Build your sequence in init()
 */
@Autonomous(name = "Blank Auton (Template)", group = "Templates")
public class BlankAuton extends AutonTemplate {

    // ============================================================
    // OPTION 1: Visualizer Paths (uncomment and paste export)
    // ============================================================
    // private VisualizerPaths paths;

    // ============================================================
    // OPTION 2: Named Paths (uncomment if needed)
    // ============================================================
    // private Path customPath;
    // private PathChain complexPath;

    @Override
    protected void buildPaths() {
        // Set starting pose (x, y, heading in radians)
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        // OPTION 1: Instantiate visualizer paths
        // paths = new VisualizerPaths(follower);

        // OPTION 2: Build custom named paths

        // Bezier curve path:
        // customPath = new Path(new BezierCurve(
        //     new Pose(0, 0),
        //     new Pose(12, 12),  // control point
        //     new Pose(24, 0)
        // ));
        // customPath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90));

        // Or straight line path:
        // complexPath = follower.pathBuilder()
        //     .addPath(new BezierLine(new Pose(0, 0), new Pose(24, 24)))
        //     .setTangentHeadingInterpolation()
        //     .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        // Not used - framework handles execution
    }

    @Override
    public void init() {
        super.init();

        // ============================================================
        // BUILD YOUR AUTONOMOUS SEQUENCE HERE
        // ============================================================
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)

                // Example: Initial actions
                // .parallel(p -> p.limelightScan().catalog())
                // .shoot()

                // Example: Inline coordinate movement (fastest)
                // Straight line with tangent heading:
                // .moveToTangent(0, 0, 24, 24)

                // Straight line with heading control:
                // .moveTo(24, 24, 0, 48, 48, 90)

                // Bezier curve with control point:
                // .moveToViaCurve(0, 0, 0, 12, 12, 24, 0, 90)
                //                 start  ctrl    end

                // Example: Visualizer path
                // .moveTo(paths.Path1)

                // Example: Named path
                // .moveTo(customPath)

                // Example: Parallel actions
                // .parallel(p -> p.moveToTangent(48, 48, 60, 60).intakeStart())
                // .intakeStop()

                .build();
    }

    // ============================================================
    // OPTION 1: Paste visualizer export here (uncomment to use)
    // ============================================================
    /*
    private static class VisualizerPaths {
        public PathChain Path1;
        public PathChain Path2;

        public VisualizerPaths(Follower follower) {
            // Paste exported paths from Pedro Pathing Visualizer here

            // Straight line example:
            // Path1 = follower.pathBuilder()
            //     .addPath(new BezierLine(new Pose(0, 0), new Pose(24, 24)))
            //     .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
            //     .build();

            // Bezier curve example:
            // Path2 = follower.pathBuilder()
            //     .addPath(new BezierCurve(
            //         new Pose(24, 24),
            //         new Pose(36, 12),  // control point
            //         new Pose(48, 24)
            //     ))
            //     .setTangentHeadingInterpolation()
            //     .build();
        }
    }
    */
}
