package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import utility.AutonTemplate;

@Autonomous(name = "BlueFrontAuton", group = "Autonomous")
public class BlueFrontAuton extends AutonTemplate {
    private Path startToShoot;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(28, 72, Math.toRadians(180));

    private final Pose shootPose = new Pose(54,90, Math.toRadians(135));
    private final Pose firstPickupPose = new Pose(24,86, Math.toRadians(180));
    private final Pose secondPickupPose = new Pose(24,61, Math.toRadians(180));

    private final Pose shootToFirstControlPoint = new Pose(66.5,78.5);
    private final Pose shootToSecondControlPoint = new Pose(69.7,49.5);

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToFirstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToSecondControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
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
                if (follower.isBusy())
                    break;
                follower.followPath(startToShoot, true);
                setPathState(1);
                break;

            case 1:
                if (follower.isBusy())
                    break;
                startAutonShoot();
                follower.followPath(shootToFirst, true);
                setPathState(2);
                follower.setMaxPower(.5);
                runAutonIntake();
                break;

            case 2:
                if (follower.isBusy())
                    break;
                stopAutonIntake();
                follower.followPath(firstToShoot, true);
                follower.setMaxPower(1);
                setPathState(3);
                addToSpindexer();
                break;

            case 3:
                if (follower.isBusy())
                    break;
                startAutonShoot();
                follower.followPath(shootToSecond, true);
                setPathState(4);
                follower.setMaxPower(.5);
                runAutonIntake();
                break;

            case 4:
                if (follower.isBusy())
                    break;
                follower.followPath(secondToShoot, true);
                stopAutonIntake();
                setPathState(5);
                follower.setMaxPower(1);
                addToSpindexer();
                break;

            case 5:
                if (follower.isBusy())
                    break;
                startAutonShoot();
                follower.followPath(shootToStop, true);
                setPathState(6);
                break;
        }
    }

    @Override
    public void init() {
        super.init();
    }
}













