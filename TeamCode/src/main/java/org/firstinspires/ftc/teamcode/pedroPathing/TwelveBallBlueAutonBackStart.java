package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Intake.IntakeMotors;

//This is for future use

@Autonomous(name = "TwelveBallBackBlueAuton")
public class TwelveBallBlueAutonBackStart extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path startToScan;

    private PathChain
            scanToShoot,
            shootToFirst,
            firstToShoot,
            shootToSecond,
            secondToShoot,
            shootToThird,
            thirdToShoot,
            shootToStop;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(126, 12, Math.toRadians(90));

    private final Pose scanPose = new Pose(57.5,113.5, Math.toRadians(110));
    private final Pose shootPose = new Pose(54,90, Math.toRadians(135));
    private final Pose firstPickupPose = new Pose(24,83.5, Math.toRadians(180));
    private final Pose secondPickupPose = new Pose(24,58.5, Math.toRadians(180));
    private final Pose thirdPickupPose = new Pose(24, 33.5, Math.toRadians(180));

    private final Pose shootToFirstControlPoint = new Pose(58.5,74);
    private final Pose shootToSecondControlPoint = new Pose(65,72);
    private final Pose shootToSecondControlPoint2 = new Pose(77,57);
    private final Pose shootToThirdControlPoint = new Pose(80, 23);

    IntakeMotors intakeMotors = new IntakeMotors();

    public void buildPaths(){
        startToScan = new Path(new BezierLine(startPose, scanPose));
        startToScan.setLinearHeadingInterpolation(startPose.getHeading(), scanPose.getHeading());

        scanToShoot = follower.pathBuilder()

                .addPath(new BezierLine(scanPose, shootPose))
                .setLinearHeadingInterpolation(scanPose.getHeading(), shootPose.getHeading())
                .build();
        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToFirstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .build();

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose,shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();
        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToSecondControlPoint, shootToSecondControlPoint2 ,secondPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPose.getHeading())
                .build();
        secondToShoot = follower.pathBuilder()
                .addPath(new BezierLine(secondPickupPose, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .build();
        shootToThird = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToThirdControlPoint, thirdPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdPickupPose.getHeading())
                .build();
        thirdToShoot = follower.pathBuilder()
                .addPath(new BezierLine(thirdPickupPose, shootPose))
                .setLinearHeadingInterpolation(thirdPickupPose.getHeading(), shootPose.getHeading())
                .build();
        shootToStop = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int i){
        pathState = i;
        pathTimer.resetTimer();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startToScan);
                setPathState(1);

                break;

            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (follower.isBusy()) {
                    break;
                }
                    /* Score Preload */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(scanToShoot, true);
                setPathState(2);

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(follower.isBusy()) {
                    break;
                }
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                intakeMotors.toggleIntake();
                follower.followPath(shootToFirst,true);
                setPathState(3);

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.isBusy()) {
                    break;
                }
                    /* Score Sample *//*
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(firstToShoot,true);
                setPathState(4);
                intakeMotors.toggleIntake();

            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(follower.isBusy()) {
                    break;
                }
                    /* Grab Sample *//*
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                intakeMotors.toggleIntake();
                follower.followPath(shootToSecond,true);
                setPathState(5);

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(follower.isBusy()) {
                    break;
                }
                    /* Score Sample *//*
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                follower.followPath(secondToShoot,true);
                setPathState(6);
                intakeMotors.toggleIntake();

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(follower.isBusy()) {
                    break;
                }
                intakeMotors.toggleIntake();
                    /* Grab Sample *//*
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                follower.followPath(shootToThird, true);
                setPathState(7);

            case 7:
                if(follower.isBusy()) {
                    break;
                }
                follower.followPath(thirdToShoot, true);
                setPathState(8);

            case 8:
                if(follower.isBusy()) {
                    break;
                }
                intakeMotors.toggleIntake();
                follower.followPath(shootToStop);
                setPathState(9);
        }
    }

    @Override
    public void loop () {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init () {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


        intakeMotors.initIntake(hardwareMap.get(DcMotorEx.class, "inMotor"));
    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop () {
    }
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start () {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop () {
    }
}
