package Autos;

import Constants.EnumConstants;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants;
import subsystems.Shooter;
import subsystems.Turret;
import subsystems.Odometry;
import subsystems.Intake;
import subsystems.Spindexer;
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
    protected ShootingValidator shootingValidator;

    /** The autonomous command sequence built by subclasses */
    protected Command autonomousCommand;

    /**
     * Set the current path state and reset the path timer
     */
    protected void setPathState(int state) {
        pathState = state;
        pathTimer.resetTimer();
    }

    /**
     * Subclasses must implement this to build paths and create the autonomousCommand.
     */
    protected abstract void buildPaths();

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
        spindexer.resetToEmptyPosition();
        limelight = new subsystems.Limelight();

        // Link Turret to Shooter for distance calculations
        shooter.setTurret(turret);

        // Link Odometry to Shooter for field state
        shooter.setOdometry(odometry);

        shootingValidator = new ShootingValidator(odometry, telemetry);

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret, odometry, spindexer, limelight);

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

        // Schedule the autonomous command
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }

        limelight.resetLimelight();
        limelight.setMode(EnumConstants.LimelightMode.TagTracking);
    }

    @Override
    public void loop() {
        // Update follower FIRST (before commands run)
        follower.update();

        // Update sensors BEFORE commands execute to avoid race condition
        robotHardware.intakeSensorPair.update();
        robotHardware.transferSensorPair.update();
        robotHardware.rampSensorPair.update();

        // Run the command scheduler
        CommandScheduler.getInstance().run();

        // Display autonomous telemetry
        if (autonomousCommand != null) {
            telemetry.addData("Auto Status", autonomousCommand.isFinished() ? "Finished" : "Running");
            telemetry.addData("Follower Busy", follower.isBusy());

            // DEBUG: Show follower position for debugging
            telemetry.addLine("--- POSITION DEBUG ---");
            telemetry.addData("Position", "X:%.1f Y:%.1f H:%.1f",
                follower.getPose().getX(), follower.getPose().getY(),
                Math.toDegrees(follower.getPose().getHeading()));

            telemetry.addData("Limelight Mode", limelight.getCurrentMode());
            telemetry.addData("Motif Detected", limelight.isMotifDetected());
        }

        // Run subsystem periodic methods
        shooter.periodic();
        turret.periodic();
        odometry.periodic();
        spindexer.periodic();
        intake.periodic();
        limelight.periodic();

        telemetry.update();
    }


    @Override
    public void stop() {
        OdometryConstants.endingAutonPose = follower.getPose();
        // Clean up the command scheduler
        CommandScheduler.getInstance().reset();
    }
}