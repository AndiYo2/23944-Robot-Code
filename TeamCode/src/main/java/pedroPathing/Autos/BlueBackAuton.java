package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import pedroPathing.Constants;
import subsystems.*;
import utility.RobotHardware;


@Autonomous(name = "BlueBackAuton", group = "Autonomous")
public class BlueBackAuton extends OpMode {
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    RobotHardware robotHardware;

    Shooter shooter;
    Intake intake;
    Spindexer spindexer;

    private Path startToFirst;

    private PathChain
            firstToShoot,
            shootToSecondOne,
            shootToSecondTwo,
            secondToShoot,
            shootToStop;

    private final Pose startPose = new Pose(57, 9, Math.toRadians(90));
    private final Pose endPose = new Pose(57, 45, Math.toRadians(90));

    private final Pose shootPose = new Pose(57, 11, Math.toRadians(90));
    private final Pose firstPickupPose = new Pose(20,38, Math.toRadians(180));
    private final Pose secondPickupPoseOne = new Pose(134,37.5, Math.toRadians(270));
    private final Pose secondPickupPoseTwo = new Pose(134,10, Math.toRadians(270));

    private final Pose shootToFirstControlPoint = new Pose(66.5,31);
    private final Pose shootToSecondControlPoint = new Pose(77,61);

    private final Pose secondtoShootControlPoint = new Pose(115,45);


    public void buildPaths(){
        startToFirst = new Path(new BezierCurve(startPose, shootToFirstControlPoint, firstPickupPose));
        startToFirst.setLinearHeadingInterpolation(startPose.getHeading(), firstPickupPose.getHeading());

        firstToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(firstPickupPose, shootToFirstControlPoint, shootPose))
                .setLinearHeadingInterpolation(firstPickupPose.getHeading(), shootPose.getHeading())
                .build();
//        shootToSecondOne = follower.pathBuilder()
//                .addPath(new BezierCurve(shootPose, shootToSecondControlPoint, secondPickupPoseOne))
//                .setLinearHeadingInterpolation(shootPose.getHeading(), secondPickupPoseOne.getHeading())
//                .build();
//        shootToSecondTwo = follower.pathBuilder()
//                .addPath(new BezierLine(secondPickupPoseOne, secondPickupPoseTwo))
//                .setLinearHeadingInterpolation(secondPickupPoseOne.getHeading(), secondPickupPoseTwo.getHeading())
//                .build();
//        secondToShoot = follower.pathBuilder()
//                .addPath(new BezierCurve(secondPickupPoseTwo, secondtoShootControlPoint, shootPose))
//                .setLinearHeadingInterpolation(secondPickupPoseTwo.getHeading(), shootPose.getHeading())
//                .build();
        shootToStop = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void setPathState(int i){
        pathState = i;
        pathTimer.resetTimer();
    }
    // Replace wait() and action methods with these self-contained loops:

    public void wait(double time){
        actionTimer.resetTimer();
        while(actionTimer.getElapsedTimeSeconds() < time){
            // Keep subsystems and follower updating during the wait
            follower.update();
            shooter.periodic();
            spindexer.periodic();
            intake.periodic();
        }
    }

    public void autonShoot(){
        for(int i = 0; i < 3; i++) {
            wait(.3);
            spindexer.flickBallOut();
            wait(.2);

            shooter.shootBall();
            wait(.2);

            spindexer.rotate();
            wait(.5);
        }
    }

    public void addToSpindexer(){
        runAutonIntake();

        spindexer.rotate();
        wait(.5);

        spindexer.rotate();
        wait(.5);

        stopAutonIntake();
    }

    public void runAutonIntake(){
        intake.setIntakePower(1);
        intake.setStagingMotorPower(1);
    }

    public void stopAutonIntake(){
        intake.setIntakePower(0);
        intake.setStagingMotorPower(0);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                spindexer.rotate();
                wait(1.5);



                autonShoot();

                setPathState(1);
            case 1:
                if (follower.isBusy())
                    break;
                runAutonIntake();
                follower.followPath(startToFirst, true);
                setPathState(2);
                follower.setMaxPower(.5);

            case 2:
                if (follower.isBusy())
                    break;
                follower.setMaxPower(.75);
                stopAutonIntake();
                follower.followPath(firstToShoot, true);
                setPathState(3);
                addToSpindexer();

            case 3:
                if(follower.isBusy())
                    break;
                autonShoot();

                follower.followPath(shootToStop, true);
                setPathState(4);
        }
    }
    /** This method is called once at the init of the OpMode. **/

    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Shooter Power", shooter.getShooterPower());
        telemetry.addData("Shooter distance", shooter.getDistanceToTarget());
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
        actionTimer = new Timer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();

        robotHardware = RobotHardware.getInstance();
        robotHardware.init(hardwareMap);
        follower.setStartingPose(startPose);

        shooter = new Shooter();
        intake = new Intake();
        spindexer = new Spindexer();
        shooter.disableLimelight();




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