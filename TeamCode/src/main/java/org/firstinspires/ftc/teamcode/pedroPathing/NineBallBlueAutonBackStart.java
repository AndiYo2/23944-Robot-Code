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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "BackBlueAuton")
public class NineBallBlueAutonBackStart extends OpMode{
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private Path startToShoot;

    private PathChain
            shootToFirst,
            firstToShoot,
            shootToSecond,
            secondToShoot,
            shootToStop;

    private final Pose startPose = new Pose(63, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(18, 12, Math.toRadians(90));

    private final Pose shootPose = new Pose(54,90, Math.toRadians(135));
    private final Pose firstPickupPose = new Pose(24,83.5, Math.toRadians(180));
    private final Pose secondPickupPose = new Pose(24,60, Math.toRadians(180));

    private final Pose shootToFirstControlPoint = new Pose(58.5,74);
    private final Pose shootToSecondControlPoint = new Pose(63,72);
    private final Pose shootToSecondControlPoint2 = new Pose(70,57);

    IntakeMotors intakeMotors = new IntakeMotors();
    OuttakeMotors outtakeMotors = new OuttakeMotors();

    public void buildPaths(){
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

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


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (follower.isBusy())
                    break;
                follower.followPath(startToShoot, true);
                outtakeMotors.shootAuton();
                setPathState(1);

            case 1:
                if(follower.isBusy()|| outtakeMotors.flywheelRunning())
                    break;
                intakeMotors.toggleIntake(1);
                follower.followPath(shootToFirst,true);
                follower.setMaxPower(.5);
                setPathState(2);

            case 2:
                if(follower.isBusy())
                    break;
                intakeMotors.stopIntakeBall();
                follower.followPath(firstToShoot,true);
                follower.setMaxPower(1);
                outtakeMotors.shootAuton();
                setPathState(3);
                
            case 3:
                if(follower.isBusy() || outtakeMotors.flywheelRunning())
                    break;
                intakeMotors.toggleIntake(1);
                follower.followPath(shootToSecond,true);
                follower.setMaxPower(.5);
                setPathState(4);

            case 4:
                if(follower.isBusy())
                    break;
                intakeMotors.stopIntakeBall();
                follower.followPath(secondToShoot,true);
                follower.setMaxPower(1);
                outtakeMotors.shootAuton();
                setPathState(5);

            case 5:
                if(follower.isBusy() || outtakeMotors.flywheelRunning())
                    break;
                follower.followPath(shootToStop, true);
                setPathState(6);
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
