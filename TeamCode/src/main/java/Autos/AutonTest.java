package Autos;

import com.arcrobotics.ftclib.command.Subsystem;
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


@Autonomous(name = "BackRedAutonTest", group = "Autonomous")
public class AutonTest extends OpMode {
    private Follower follower;

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    RobotHardware robotHardware;

    Shooter shooter;
    Intake intake;
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
    private boolean isWaiting = false;
    private double waitDuration = 0;

    public void startWait(double time){
        actionTimer.resetTimer();
        waitDuration = time;
        isWaiting = true;
    }

    public boolean isWaitComplete(){
        if (!isWaiting) {
            return true;
        }

        if (actionTimer.getElapsedTimeSeconds() >= waitDuration) {
            isWaiting = false;
            return true;
        }

        return false;
    }

    // Shooting state machine
    private int shootStep = 0;
    private int ballsShot = 0;

    public boolean autonShoot(){
        switch(shootStep) {
            case 0:
                spindexer.flickBallOut();
                startWait(0.200);
                shootStep++;
                break;
            case 1:
                if (!isWaitComplete()) break;
                shooter.shootBall();
                startWait(0.350);
                shootStep++;
                break;
            case 2:
                if (!isWaitComplete()) break;
                spindexer.rotate();
                startWait(0.350);
                shootStep++;
                break;
            case 3:
                if (!isWaitComplete()) break;
                ballsShot++;
                if (ballsShot < 3) {
                    shootStep = 0; // Go back to shoot next ball
                } else {
                    shootStep = 0; // Reset for next use
                    ballsShot = 0;
                    return true; // Signal completion
                }
                break;
        }
        return false; // Still shooting
    }

    // Spindexer state machine
    private int spindexerStep = 0;

    public boolean addToSpindexer(){
        switch(spindexerStep) {
            case 0:
                runAutonIntake();
                spindexer.rotate();
                startWait(0.5);
                spindexerStep++;
                break;
            case 1:
                if (!isWaitComplete()) break;
                spindexer.rotate();
                startWait(0.5);
                spindexerStep++;
                break;
            case 2:
                if (!isWaitComplete()) break;
                stopAutonIntake();
                spindexerStep = 0; // Reset for next use
                return true; // Signal completion
        }
        return false; // Still running
    }

    public void runAutonIntake(){
        intake.setIntakePower(1);
        intake.setStagingMotorPower(1);
    }

    public void stopAutonIntake(){
        intake.setIntakePower(0);
        intake.setStagingMotorPower(0);
    }

    public void toggleSlowMode(){
        follower.setMaxPower(.5);
    }

    // Updated autonomousPathUpdate with proper state tracking
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (follower.isBusy())
                    break;
                spindexer.rotate();
                follower.followPath(startToShoot, true);
                setPathState(1);
                break;

            case 1:
                if (follower.isBusy())
                    break;

                // Keep calling autonShoot until it returns true
                if (!autonShoot())
                    break;

                follower.followPath(shootToFirst, true);
                setPathState(2);
                runAutonIntake();
                toggleSlowMode();
                break;

            case 2:
                if (follower.isBusy())
                    break;
                stopAutonIntake();
                follower.followPath(firstToShoot, true);
                toggleSlowMode();
                setPathState(3);
                break;

            case 3:
                if (follower.isBusy())
                    break;

                // Keep calling addToSpindexer until it returns true
                if (!addToSpindexer())
                    break;

                setPathState(4);
                break;

            case 4:
                // Keep calling autonShoot until it returns true
                if (!autonShoot())
                    break;

                follower.followPath(shootToSecond, true);
                setPathState(5);
                toggleSlowMode();
                runAutonIntake();
                break;

            case 5:
                if(follower.isBusy())
                    break;
                follower.followPath(secondToShoot, true);
                stopAutonIntake();
                setPathState(6);
                toggleSlowMode();
                break;

            case 6:
                if (follower.isBusy())
                    break;

                // Keep calling addToSpindexer until it returns true
                if (!addToSpindexer())
                    break;

                setPathState(7);
                break;

            case 7:
                // Keep calling autonShoot until it returns true
                if (!autonShoot())
                    break;

                follower.followPath(shootToStop, true);
                setPathState(8);
                break;
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