package Autos;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import utility.RobotConstants;

@Autonomous(name = "BlueBackAuton", group = "Autonomous")
public class BlueBackAuton extends AutonTemplate {
    private Path shootToFirst;
    private PathChain firstToShoot, shootToSecond, secondToShoot, secondShootToStop;

    private final Pose startPose = new Pose(56.5, 8.5, Math.toRadians(90));
    private final Pose endPose = new Pose(54.5, 40, Math.toRadians(90));

    private final Pose firstPickupPose = new Pose(23, 37.5, Math.toRadians(180));
    private final Pose firstPickupPoseControlPoint = new Pose(56, 31);
    private final Pose firstShootPose = new Pose(56.5, 15.5, 135);


    private final Pose secondPickupPose = new Pose(18, 63.5, 255);
    private final Pose secondPickupPoseControlPoint = new Pose(70.5, 60);
    private final Pose secondShootPose = new Pose(54.5, 13, 90);






    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        shootToFirst = new Path(new BezierCurve(startPose, firstPickupPoseControlPoint, firstPickupPose));
        shootToFirst.setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading());

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose, firstShootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), firstShootPose.getHeading())
                .build();

        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(firstShootPose, secondPickupPoseControlPoint, secondPickupPose))
                .setLinearHeadingInterpolation(firstShootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, secondShootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), secondShootPose.getHeading())
                .build();

        secondShootToStop = follower.pathBuilder()
                .addPath(new BezierLine(secondShootPose, endPose))
                .setLinearHeadingInterpolation(secondShootPose.getHeading(), endPose.getHeading())
                .build();
    }

    ///RUN:
    /// START LIMELIGHT SCANNING, run the turret to position, start catalogging
    ///
    /// when both are done, run shooter sequence
    /// when shooting done, shoot to first, run intake
    /// when at first, stop intake, catalog, and go to firstShootPose
    /// when all that is done, shoot
    /// when done, follow next 2 paths till shootToSecondTwo
    /// Run intake, and go to shootTOSecondThree
    /// when done, stop intake, run catalog, go to shootTWO Pose
    /// when done, shoot
    /// when done, go to end pose
    ///
    ///
    @Override
    protected void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                // Initial Setup: Start limelight tag scanning and catalog preloaded balls
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    limelight.toggleMode();
                    catalogManager.initiateCataloging();
                }

                // Wait for both limelight motif detection AND cataloging to complete
                if (!limelight.isMotifDetected() || catalogManager.getState() != RobotConstants.Enums.CatalogingCases.Idle)
                    break;

                // Both complete - start shooting preloaded balls
                sequenceManager.startShootingSequence();
                setPathState(1);
                break;

            case 1:
                // Wait for initial shooting to complete
                if (sequenceManager.isExecuting())
                    break;

                // Shooting done - move to first pickup and run intake
                follower.followPath(shootToFirst, true);
                intake.runIntake();
                setPathState(2);
                break;

            case 2:
                // Wait for path to first pickup to complete
                if (follower.isBusy())
                    break;

                // At first pickup - stop intake, start cataloging, and begin moving to shoot pose
                intake.stopIntake();
                catalogManager.initiateCataloging();
                follower.followPath(firstToShoot, true);
                setPathState(3);
                break;

            case 3:
                // Wait for BOTH path to first shoot pose AND cataloging to complete
                if (follower.isBusy() || catalogManager.getState() != RobotConstants.Enums.CatalogingCases.Idle)
                    break;

                // Both complete - start shooting
                sequenceManager.startShootingSequence();
                setPathState(4);
                break;

            case 4:
                // Wait for first shooting to complete
                if (sequenceManager.isExecuting())
                    break;

                // Shooting done - start second pickup sequence
                follower.followPath(shootToSecond, true);
                intake.runIntake();
                setPathState(5);
                break;

            case 5:
                // Wait for second pickup path 3 to complete
                if (follower.isBusy())
                    break;

                // At final pickup - stop intake, start cataloging, and begin moving to second shoot pose
                intake.stopIntake();
                catalogManager.initiateCataloging();
                follower.followPath(secondToShoot, true);
                setPathState(6);
                break;

            case 6:
                // Wait for BOTH path to second shoot pose AND cataloging to complete
                if (follower.isBusy() || catalogManager.getState() != RobotConstants.Enums.CatalogingCases.Idle)
                    break;

                // Both complete - start final shooting
                sequenceManager.startShootingSequence();
                setPathState(7);
                break;

            case 7:
                // Wait for final shooting to complete
                if (sequenceManager.isExecuting())
                    break;

                // Shooting done - move to park position
                follower.followPath(secondShootToStop, true);
                setPathState(8);
                break;

            case 8:
                // Wait for park path to complete
                if (follower.isBusy())
                    break;

                // Autonomous routine finished
                break;
        }
    }

    @Override
    public void init() {
        super.init();
    }
}