package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import utility.AutonTemplate;

@Autonomous(name = "BlueBackAuton", group = "Autonomous")
public class BlueBackAuton extends AutonTemplate {
    private Path startToFirst;
    private PathChain firstToShoot, shootToStop;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(57, 45, Math.toRadians(90));
    private final Pose shootPose = new Pose(57, 11, Math.toRadians(90));
    private final Pose firstPickupPose = new Pose(20, 38, Math.toRadians(180));
    private final Pose shootToFirstControlPoint = new Pose(66.5, 31);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToFirst = new Path(new BezierCurve(startPose, shootToFirstControlPoint, firstPickupPose));
        startToFirst.setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading());

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, shootToFirstControlPoint, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToStop = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                spindexer.rotate();
                wait(1.5);
                startAutonShoot();
                setPathState(1);
                break;

            case 1:
                if (follower.isBusy())
                    break;
                runAutonIntake();
                follower.followPath(startToFirst, true);
                setPathState(2);
                follower.setMaxPower(.5);
                break;

            case 2:
                if (follower.isBusy())
                    break;
                follower.setMaxPower(.75);
                stopAutonIntake();
                follower.followPath(firstToShoot, true);
                setPathState(3);
                addToSpindexer();
                break;

            case 3:
                if (follower.isBusy())
                    break;
                startAutonShoot();
                follower.followPath(shootToStop, true);
                setPathState(4);
                break;
        }
    }

    @Override
    public void init() {
        super.init();
    }
}