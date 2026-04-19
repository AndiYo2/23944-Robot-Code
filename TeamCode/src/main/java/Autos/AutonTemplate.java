package Autos;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.Constants;
import subsystems.Shooter;
import subsystems.Turret;
import subsystems.Intake;
import subsystems.Spindexer;
import Constants.FieldMap;
import Constants.OdometryConstants;
import Constants.ShooterConstants;
import Constants.SpindexerConstants;
import utility.FieldDrawing;
import utility.RobotHardware;
import commands.GuaranteeSortedAutoCatalogCommand;
import utility.SpindexerAndMotifStatus;
import utility.TelemetryHelper;

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
    protected Intake intake;
    protected Spindexer spindexer;
    protected subsystems.Limelight limelight;
    private TelemetryHelper telemetryHelper;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private static final double TELEMETRY_INTERVAL_MS = 200; // ~5 Hz

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
        CommandScheduler.getInstance().reset();  // Clean slate from any prior OpMode

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
        spindexer = new Spindexer();
        spindexer.resetToEmptyPosition();
        limelight = new subsystems.Limelight();

        // Set servo positions during auto init (allowed by FTC rules)
        shooter.initServoPositions();
        turret.initServoPositions();
        spindexer.initServoPositions();

        // Link Turret to Shooter for distance calculations
        shooter.setTurret(turret);

        // Link Shooter to Turret for lead-compensated aiming
        turret.setShooter(shooter);

        telemetryHelper = new TelemetryHelper();
        telemetryHelper.setSubsystems(shooter, turret, spindexer, limelight,
                null, intake);

        FieldDrawing.init();

        // Register subsystems with the command scheduler
        CommandScheduler.getInstance().registerSubsystem(intake, shooter, turret, spindexer, limelight);

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

        robotHardware.resetProgressiveScan();
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        telemetryHelper.recordLoop(loopMs);

        // Clear bulk cache so hardware reads (encoders, sensors) return fresh data
        robotHardware.clearBulkCache();

        // Update follower FIRST (before commands run)
        follower.update();
        robotHardware.updateCachedPose();
        OdometryConstants.endingAutonPose = follower.getPose();

        // Progressive scan: one sensor pair at a time (spindexer → transfer → ramp)
        robotHardware.progressivePollAuto();

        // Run the command scheduler
        CommandScheduler.getInstance().run();

        // Rate-limit telemetry sends to ~5 Hz to avoid synchronous WiFi lag spikes
        if (telemetryTimer.milliseconds() >= TELEMETRY_INTERVAL_MS) {
            telemetryTimer.reset();

            // Display autonomous telemetry on Driver Station
            if (autonomousCommand != null) {
                telemetry.addData("Auto Status", autonomousCommand.isFinished() ? "Finished" : "Running");
                telemetry.addData("Follower Busy", follower.isBusy());
                telemetry.addData("Vision Scan", commands.VisionCollectCommand.lastScanResult);
                telemetry.addData("Vision Lanes", vision.LaneSelector.lastLaneDebug);

                telemetry.addLine("--- POSITION DEBUG ---");
                telemetry.addData("Follower Pose", "X:%.1f Y:%.1f H:%.1f",
                    follower.getPose().getX(), follower.getPose().getY(),
                    Math.toDegrees(follower.getPose().getHeading()));
                telemetry.addData("Pinpoint Pose", "X:%.1f Y:%.1f H:%.1f",
                    robotHardware.cachedPoseX, robotHardware.cachedPoseY,
                    Math.toDegrees(robotHardware.cachedHeading));

                telemetry.addLine("--- SHOOTER DEBUG ---");
                telemetry.addData("Distance", "%.1f in", shooter.getDistanceToTarget());
                telemetry.addData("Target Vel", "%.0f", shooter.getTargetVelocity());
                telemetry.addData("Actual Vel", "%.0f", shooter.getCurrentVelocity());
                telemetry.addData("Hood Angle", "%.1f°", shooter.getTargetHoodAngle());
                telemetry.addData("Goal", "X:%.0f Y:%.0f",
                    FieldMap.getGoalPosition().getX(),
                    FieldMap.getGoalPosition().getY());

                telemetry.addData("Limelight Mode", limelight.getCurrentMode());
                telemetry.addData("Motif Detected", limelight.isMotifDetected());

                telemetry.addLine("--- SPINDEXER DEBUG ---");
                telemetry.addData("Spindexer Degrees", "%d°", spindexer.getTargetPosition());
                telemetry.addData("Spindexer Servo", "%.3f", spindexer.getServoPosition());
                telemetry.addData("Ball Pattern", SpindexerAndMotifStatus.SpindexerPattern.getSpindexerPatternString());
                telemetry.addData("Balls Tracked", SpindexerAndMotifStatus.SpindexerPattern.getBallCount());
                telemetry.addData("Rotation Idle", spindexer.isRotationIdle());
                telemetry.addData("Flick State", spindexer.getCurrentState());
                telemetry.addData("Shooting Mode", SpindexerConstants.currentMode);
                telemetry.addData("Catalog Debug", GuaranteeSortedAutoCatalogCommand.lastCatalogDebug);
            }

            // Panels telemetry (graph + debug) and field drawing
            telemetryHelper.update(telemetry, loopMs);
            FieldDrawing.drawFollowerDebug(follower);
        }
    }


    @Override
    public void stop() {
        follower.startTeleopDrive(true);
        // Clean up the command scheduler
        CommandScheduler.getInstance().reset();
        OdometryConstants.endingAutonPose = follower.getPose();
    }
}