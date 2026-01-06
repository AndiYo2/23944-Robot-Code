package Autos.NineBalls;

import Autos.AutonTemplate;
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
@Autonomous(name = "BlueBack9Ball", group = "Templates")
public class BlueBack9Ball extends AutonTemplate {

     private Paths paths;

    @Override
    protected void buildPaths() {
        follower.setStartingPose(new Pose(56.5, 8.5, Math.toRadians(90)));
        paths = new Paths(follower);
    }

    @Override
    protected void autonomousPathUpdate() {
    }

    @Override
    public void init() {
        super.init();
        executor = new AutonSequence(follower, intake, catalogManager, sequenceManager, limelight)

                .parallel(p -> p.limelightScan().catalog())
                .shoot()
                .parallel(p -> p.moveTo(paths.shootToFirst).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(paths.firstToShoot).catalog())
                .shoot()
                .parallel(p -> p.moveTo(paths.shootToSecond).intakeStart())
                .intakeStop()
                .parallel(p -> p.moveTo(paths.secondToShoot).catalog())
                .shoot()
                .moveTo(paths.shootToStop)
                .build();
    }

    public static class Paths {
        public PathChain shootToFirst;
        public PathChain firstToShoot;
        public PathChain shootToSecond;
        public PathChain secondToShoot;
        public PathChain shootToStop;

        public Paths(Follower follower) {
            shootToFirst = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.500, 8.500),
                                    new Pose(57.500, 30.000),
                                    new Pose(22.500, 38.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            firstToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 38.000),

                                    new Pose(56.500, 10.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            shootToSecond = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.500, 10.500),
                                    new Pose(62.800, 60.000),
                                    new Pose(22.500, 62.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            secondToShoot = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.500, 62.000),

                                    new Pose(56.500, 10.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            shootToStop = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.500, 10.500),

                                    new Pose(56.500, 40.500)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))

                    .build();
        }
    }
}
