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
import org.firstinspires.ftc.teamcode.Outtake.OuttakeMotors;

@Autonomous(name = "BackRedAuton")
public class nineBallRedAutonBackStart extends OpMode{
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
            shootToStop;

    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(126, 12, Math.toRadians(90));

    private final Pose scanPose = new Pose(86.5,113.5, Math.toRadians(110));
    private final Pose shootPose = new Pose(90,90, Math.toRadians(45));
    private final Pose firstPickupPose = new Pose(120,83.5, Math.toRadians(0));
    private final Pose secondPickupPose = new Pose(120,55.5, Math.toRadians(0));

    private final Pose shootToFirstControlPoint = new Pose(85.5,74);
    private final Pose shootToSecondControlPoint = new Pose(79,67);
    private final Pose shootToSecondControlPoint2 = new Pose(67,52);

    IntakeMotors intakeMotors = new IntakeMotors();

    OuttakeMotors outtakeMotors = new OuttakeMotors();


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
        shootToStop = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())

                .build();

    }

    public void setPathState(int i){
        pathState = i;
        pathTimer.resetTimer();
    }

            /* You could check for
               - Follower State: "if(!follower.isBusy()) {}"
               - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
               - Robot Position: "if(follower.getPose().getX() > 36) {}"
               */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(startToScan);
                setPathState(1);


                break;
            case 1:
                if (follower.isBusy())
                    break;
                follower.followPath(scanToShoot, true);
                outtakeMotors.shoot(1);
                setPathState(2);

            case 2:
                if(follower.isBusy())
                    break;
                intakeMotors.toggleIntake(1);
                follower.followPath(shootToFirst,true);
                follower.setMaxPower(.5);
                setPathState(3);

            case 3:
                if(follower.isBusy())
                    break;
                follower.followPath(firstToShoot,true);
                follower.setMaxPower(1);
                setPathState(4);
                intakeMotors.toggleIntake(1);


            case 4:
                if(follower.isBusy())
                    break;
                intakeMotors.toggleIntake(1);
                follower.followPath(shootToSecond,true);
                follower.setMaxPower(.5);
                setPathState(5);

            case 5:
                if(follower.isBusy())
                    break;
                follower.followPath(secondToShoot,true);
                follower.setMaxPower(1);
                setPathState(6);

            case 6:
                if(follower.isBusy())
                    break;
                intakeMotors.toggleIntake(1);
                follower.followPath(shootToStop, true);
                setPathState(7);
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

            intakeMotors.initIntake(hardwareMap.get(DcMotorEx.class,"inMotor"));
            outtakeMotors.initOuttake(hardwareMap.get(DcMotorEx.class,"outMotor1"));
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