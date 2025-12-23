package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import utility.AutonTemplate;
import utility.RobotConstants;

@Autonomous(name = "RedFrontAuton", group = "Autonomous")
public class RedFrontAuton extends AutonTemplate {
    private Path startToShoot;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot, shootToStop;

    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(116, 72, Math.toRadians(0));
    private final Pose shootPose = new Pose(90, 90, Math.toRadians(50));
    private final Pose secondaryShootPose = new Pose(90, 90, Math.toRadians(45));
    private final Pose firstPickupPose = new Pose(120, 81, Math.toRadians(0));
    private final Pose secondPickupPose = new Pose(120, 53, Math.toRadians(0));
    private final Pose shootToFirstControlPoint = new Pose(85.5, 74);
    private final Pose shootToSecondControlPoint = new Pose(82, 55);

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
                .addPath(new BezierLine(firstPickupPose, secondaryShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), secondaryShootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(secondaryShootPose, shootToSecondControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(secondaryShootPose.getHeading(), secondPickupPose.getHeading())
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
                spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
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