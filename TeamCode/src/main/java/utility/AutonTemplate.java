package utility;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;
import subsystems.*;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotConstants.Enums.ShooterCases;

/**
 * Base template for all autonomous OpModes.
 * Contains common functionality for path following, timing, and subsystem control.
 */
public abstract class AutonTemplate extends OpMode {
    protected Follower follower;
    protected Timer pathTimer, actionTimer, opmodeTimer;
    protected int pathState;

    protected int ballsToShoot;

    protected RobotHardware robotHardware;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;
    protected ShooterCases shootCases;

    /**
     * Set the current path state and reset the path timer
     */
    protected void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }


    /**
     * Wait for a specified amount of time while keeping subsystems updated
     */
    protected void wait(double time) {
        actionTimer.resetTimer();
        while (actionTimer.getElapsedTimeSeconds() < time) {
            follower.update();
            shooter.periodic();
            spindexer.periodic();
            intake.periodic();
        }
    }

    /**
     * Execute an autonomous shooting sequence (3 balls)
     */
    protected void startAutonShoot() {
        ballsToShoot = 3;
        shootCases = ShooterCases.Start;
        follower.pausePathFollowing();
    }
    private void checkAutonShoot(){
        if(ballsToShoot > 0)
            shootCases = ShooterCases.Start;
        else{
            follower.resumePathFollowing();
        }
    }



    protected void shootingPeriodic(){
        switch (shootCases) {
            case Idle:
                break;
            case Start:
                if (shooter.getFlipperState() == FlickState.Retracted){
                    shootCases = ShooterCases.SpindexerFlicking;
                    spindexer.flickBallOut();
                }
                break;
            case SpindexerFlicking:
                if(spindexer.getFlipperState() == FlickState.Extended){
                    shootCases = ShooterCases.ShooterFlicking;
                    shooter.shootBall();
                }
                break;
            case ShooterFlicking:
                if(spindexer.getFlipperState() == FlickState.Retracted){
                    shootCases = ShooterCases.SpindexerRotating;
                    spindexer.rotate(120);
                }
                break;
            case SpindexerRotating:
                if(spindexer.isDoneRotating()){
                    shootCases = ShooterCases.Idle;
                    ballsToShoot--;
                    checkAutonShoot();
                }
                break;
        }
    }

    /**
     * Add balls to the spindexer from intake
     */
    protected void addToSpindexer() {
        runAutonIntake();

        spindexer.rotate(120);
        wait(.5);

        spindexer.rotate(120);
        wait(.5);

        stopAutonIntake();
    }

    /**
     * Start intake motors for autonomous
     */
    protected void runAutonIntake() {
        intake.runIntake();
    }

    /**
     * Stop intake motors
     */
    protected void stopAutonIntake() {
        intake.stopIntake();
    }

    /**
     * Build paths - must be implemented by subclass
     */
    protected abstract void buildPaths();

    /**
     * Update autonomous path state machine - must be implemented by subclass
     */
    protected abstract void autonomousPathUpdate();

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        actionTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        robotHardware = RobotHardware.getInstance();
        robotHardware.init(hardwareMap);

        shooter = new Shooter();
        intake = new Intake();
        spindexer = new Spindexer();
        shootCases = ShooterCases.Idle;

        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Common telemetry
        telemetry.addData("Shooter Power", shooter.getShooterPower());
        telemetry.addData("Shooter distance", shooter.getDistanceToTarget());
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();


        shootingPeriodic();
        shooter.periodic();
        spindexer.periodic();
        intake.periodic();
    }

    @Override
    public void stop() {}
}