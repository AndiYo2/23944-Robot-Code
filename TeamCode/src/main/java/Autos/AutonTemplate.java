package Autos;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import framework.ActionExecutor;
import pedroPathing.Constants;
import subsystems.Shooter;
import subsystems.Turret;
import subsystems.Odometry;
import subsystems.Intake;
import subsystems.Spindexer;
import utility.managers.SpindexerManager;
import Constants.OdometryConstants;
import utility.RobotHardware;
import utility.ShootingValidator;

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
    protected Turret turret;
    protected Odometry odometry;
    protected Intake intake;
    protected Spindexer spindexer;
    protected subsystems.Limelight limelight;
    protected SpindexerManager spindexerManager;
    protected ShootingValidator shootingValidator;

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
        turret = new Turret();
        odometry = new Odometry();
        spindexer = new Spindexer();
        limelight = new subsystems.Limelight();

        // Link Turret to Shooter for distance calculations
        shooter.setTurret(turret);

        // Link Odometry to Shooter for field state
        shooter.setOdometry(odometry);

        // Link Limelight subsystem to Turret for dual-mode tracking
        turret.setLimelightSubsystem(limelight);
        shootingValidator = new ShootingValidator(odometry, telemetry);
        spindexerManager = new SpindexerManager(
                spindexer,
                shooter,
                intake,
                telemetry,
                robotHardware.intakeSensorPair,
                robotHardware.transferSensorPair,
                robotHardware.rampSensorPair
        );

        buildPaths();
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);

        // Activate all PIDF controllers for proper path following
        follower.activateAllPIDFs();

        if (executor != null) {
            executor.start();
        }
        limelight.resetLimelight();
    }

    @Override
    public void loop() {
        follower.update();

        if (executor != null) {
            executor.update();

            // Display autonomous sequence telemetry
            telemetry.addData("Auto Status", executor.getStatusString());
            telemetry.addData("Sequence Executing", executor.isExecuting());
            telemetry.addData("Follower Busy", follower.isBusy());

            // DEBUG: Show follower position for debugging
            telemetry.addLine("--- DECEL DEBUG ---");
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("Manager State", spindexerManager.getStatus());
            telemetry.addData("Limelight Mode", limelight.getCurrentMode());
            telemetry.addData("Motif Detected", limelight.isMotifDetected());
        } else {
            autonomousPathUpdate();
        }

        shooter.periodic();
        turret.periodic();
        odometry.periodic();
        spindexer.periodic();
        intake.periodic();
        limelight.periodic();

        spindexerManager.update();

        telemetry.update();
    }


    @Override
    public void stop() {
        OdometryConstants.endingAutonPose = follower.getPose();
    }
}