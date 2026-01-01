package Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;
import subsystems.Shooter;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.RobotConstants;
import utility.RobotConstants.Enums.BallColor;
import utility.RobotConstants.Enums.FlickState;
import utility.RobotConstants.Enums.ShooterCases;
import utility.RobotHardware;

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
                if (shooter.getCurrentState() == FlickState.Retracted){
                    shootCases = ShooterCases.SpindexerFlicking;
                    spindexer.triggerFlick();
                }
                break;
            case SpindexerFlicking:
                if(spindexer.getCurrentState() == FlickState.Extended){
                    shootCases = ShooterCases.ShooterFlicking;
                    shooter.triggerShot();
                }
                break;
            case ShooterFlicking:
                if(spindexer.getCurrentState() == FlickState.Retracted){
                    shootCases = ShooterCases.SpindexerRotating;
                    spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
                    rotateBallPatternForward();
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

        spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
        rotateBallPatternForward();
        wait(.5);

        spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
        rotateBallPatternForward();
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
     * Catalog a ball in autonomous mode
     * Call this after running intake to catalog the ball and rotate the spindexer
     */
    protected void catalogBallAuton() {
        // Refresh color sensor reading
        robotHardware.intakeSensor.refreshScan();

        // Get detected ball color
        BallColor detectedColor = robotHardware.intakeSensor.getBallColor();

        // Only catalog if we detected an actual ball (not None)
        if (detectedColor != BallColor.None) {
            // Catalog the ball at the intake slot (slot 0)
            spindexer.catalogBall(0, detectedColor);

            // Rotate to next slot
            spindexer.rotateBy(RobotConstants.Spindexer.ROTATION_FORWARD);
            rotateBallPatternForward();

            // Wait for rotation to complete
            while (!spindexer.isDoneRotating()) {
                follower.update();
                shooter.periodic();
                spindexer.periodic();
                intake.periodic();
            }
        }
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
        telemetry.addData("Shooter Power", shooter.getFlywheelPower());
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

    /**
     * Rotates the ball pattern forward to match physical spindexer rotation.
     * When spindexer rotates forward 120°:
     * - What was in slot 2 (storage) is now in slot 0 (intake)
     * - What was in slot 0 (intake) is now in slot 1 (shooter)
     * - What was in slot 1 (shooter) is now in slot 2 (storage)
     */
    protected void rotateBallPatternForward() {
        RobotConstants.Enums.BallColor slot0 = robotHardware.spindexerPattern.getBallInSlotX(0);
        RobotConstants.Enums.BallColor slot1 = robotHardware.spindexerPattern.getBallInSlotX(1);
        RobotConstants.Enums.BallColor slot2 = robotHardware.spindexerPattern.getBallInSlotX(2);

        robotHardware.spindexerPattern.setBallPattern(slot2, slot0, slot1);
    }

    /**
     * Rotates the ball pattern backward to match physical spindexer rotation.
     * When spindexer rotates backward 120°:
     * - What was in slot 1 (shooter) is now in slot 0 (intake)
     * - What was in slot 2 (storage) is now in slot 1 (shooter)
     * - What was in slot 0 (intake) is now in slot 2 (storage)
     */
    protected void rotateBallPatternBackward() {
        RobotConstants.Enums.BallColor slot0 = robotHardware.spindexerPattern.getBallInSlotX(0);
        RobotConstants.Enums.BallColor slot1 = robotHardware.spindexerPattern.getBallInSlotX(1);
        RobotConstants.Enums.BallColor slot2 = robotHardware.spindexerPattern.getBallInSlotX(2);

        robotHardware.spindexerPattern.setBallPattern(slot1, slot2, slot0);
    }

    @Override
    public void stop() {
        RobotConstants.UpdatableConstants.endingAutonPose = follower.getPose();
    }
}