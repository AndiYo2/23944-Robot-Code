package Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import framework.ActionExecutor;
import pedroPathing.Constants;
import subsystems.Shooter;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.CatalogManager;
import utility.RobotConstants;
import utility.RobotHardware;
import utility.Shooting.ShootingSequenceManager;
import utility.Shooting.ShootingValidator;

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
    protected subsystems.Limelight limelight;
    protected ShootingSequenceManager sequenceManager;
    protected ShootingValidator shootingValidator;

    protected CatalogManager catalogManager;

    protected ActionExecutor executor;

    /**
     * Set the current path state and reset the path timer
     */
    protected void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }
    protected abstract void buildPaths();
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

        intake = new Intake();
        shooter = new Shooter();
        spindexer = new Spindexer();
        limelight = new subsystems.Limelight();

        shooter.setLimelightSubsystem(limelight);
        sequenceManager = new ShootingSequenceManager(spindexer, shooter);
        shootingValidator = new ShootingValidator(shooter, telemetry);
        catalogManager = new CatalogManager(spindexer, intake, telemetry, robotHardware.intakeSensorPair);



        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        if (executor != null) {
            executor.start();
        }
    }

    @Override
    public void loop() {
        follower.update();

        if (executor != null) {
            executor.update();
        } else {
            autonomousPathUpdate();
        }

        shooter.periodic();
        spindexer.periodic();
        intake.periodic();
        limelight.periodic();


        sequenceManager.update();
        catalogManager.update();
    }


    @Override
    public void stop() {
        RobotConstants.UpdatableConstants.endingAutonPose = follower.getPose();
    }
}