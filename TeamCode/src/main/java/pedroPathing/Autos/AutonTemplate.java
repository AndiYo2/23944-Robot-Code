package pedroPathing.Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;
import subsystems.*;
import utility.RobotHardware;

/**
 * Base template for all autonomous OpModes.
 * Contains common functionality for path following, timing, and subsystem control.
 */
public abstract class AutonTemplate extends OpMode {
    protected Follower follower;
    protected Timer pathTimer, actionTimer, opmodeTimer;
    protected int pathState;

    protected RobotHardware robotHardware;
    protected Shooter shooter;
    protected Intake intake;
    protected Spindexer spindexer;

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
     * Execute autonomous shooting sequence (3 balls)
     */
    protected void autonShoot() {
        for (int i = 0; i < 3; i++) {
            wait(.3);
            spindexer.flickBallOut();
            wait(.2);

            shooter.shootBall();
            wait(.2);

            spindexer.rotate();
            wait(.5);
        }
    }

    /**
     * Add balls to the spindexer from intake
     */
    protected void addToSpindexer() {
        runAutonIntake();

        spindexer.rotate();
        wait(.5);

        spindexer.rotate();
        wait(.5);

        stopAutonIntake();
    }

    /**
     * Start intake motors for autonomous
     */
    protected void runAutonIntake() {
        intake.setIntakePower(1);
        intake.setStagingMotorPower(1);
    }

    /**
     * Stop intake motors
     */
    protected void stopAutonIntake() {
        intake.setIntakePower(0);
        intake.setStagingMotorPower(0);
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

        shooter.periodic();
        spindexer.periodic();
        intake.periodic();
    }

    @Override
    public void stop() {}
}