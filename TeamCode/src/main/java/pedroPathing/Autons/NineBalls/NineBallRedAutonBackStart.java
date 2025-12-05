package pedroPathing.Autons.NineBalls;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.Constants;
import subsystems.*;
import utility.RobotHardware;


@Autonomous(name = "BackRedAuton")
public class NineBallRedAutonBackStart extends OpMode {
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    RobotHardware robotHardware;

    Shooter shooter;
    Intake intake;
    ColorSensorSubsytem colorSensorSubsytem;
    Spindexer spindexer;

    private Path startToShoot;

    private PathChain
            shootToFirst,
            firstToShoot,
            shootToSecond,
            secondToShoot,
            shootToStop;

    private final Pose startPose = new Pose(87, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(116, 72, Math.toRadians(0));

    private final Pose shootPose = new Pose(90,90, Math.toRadians(45));
    private final Pose secondaryShootPose = new Pose(90,90, Math.toRadians(40));
    private final Pose firstPickupPose = new Pose(120,83.5, Math.toRadians(0));
    private final Pose secondPickupPose = new Pose(120,55.5, Math.toRadians(0));

    private final Pose shootToFirstControlPoint = new Pose(85.5,74);
    private final Pose shootToSecondControlPoint = new Pose(82,55);


    public void buildPaths(){
        startToShoot = new Path(new BezierLine(startPose, shootPose));
        startToShoot.setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading());

        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, shootToFirstControlPoint, firstPickupPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstPickupPose.getHeading())
                .build();
        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstPickupPose,secondaryShootPose))
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

    public void setPathState(int i){
        pathState = i;
        pathTimer.resetTimer();
    }
    public void wait(double time){
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds() < time){
        }
    }

    public void autonShoot(){
        for(int i = 0; i < 3; i++) {
            spindexer.flickBallOut();
            wait(.200);
            shooter.shootBall();
            wait(.350);
            spindexer.rotate();
            wait(.350);
        }
    }

    public void runAutonIntake(){
        intake.setIntakePower(1);
        intake.setStagingMotorPower(1);
    }
    public void stopAutonIntake(){
        intake.setIntakePower(0);
        intake.setStagingMotorPower(0);
    }

    public void addToSpindexer(){
        runAutonIntake();
        spindexer.rotate();
        spindexer.rotate();
        wait(1000.0);
        stopAutonIntake();


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (follower.isBusy())
                    break;
                follower.followPath(startToShoot, true);
                setPathState(1);

            case 1:
                if (follower.isBusy())
                    break;

                autonShoot();

                //wait till completion
                follower.followPath(shootToFirst, true);
                setPathState(2);
                runAutonIntake();

            case 2:
                if (follower.isBusy())
                    break;
                stopAutonIntake();
                follower.followPath(firstToShoot, true);

                setPathState(3);
                addToSpindexer();

            case 3:
                if (follower.isBusy()){
                    break;
                }

                autonShoot();

                follower.followPath(shootToSecond,true);
                setPathState(4);

                runAutonIntake();


            case 4:
                if(follower.isBusy()) {
                    break;
                }
                follower.followPath(secondToShoot,true);
                stopAutonIntake();
                setPathState(5);
                addToSpindexer();

            case 5:
                if(follower.isBusy())
                    break;
                autonShoot();


                follower.followPath(shootToStop, true);
                setPathState(6);
        }
    }
    /** This method is called once at the init of the OpMode. **/

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

        shooter.periodic();
        spindexer.periodic();
        intake.periodic();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        robotHardware = new RobotHardware();
        robotHardware.init(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new Shooter();
        intake = new Intake();
        colorSensorSubsytem = new ColorSensorSubsytem();
        spindexer = new Spindexer();


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

}