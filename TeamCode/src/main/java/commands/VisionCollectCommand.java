package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import utility.RobotHardware;
import vision.ArtifactDetector;
import vision.BallLocalizer;
import vision.ChosenPath;
import vision.Detection;
import vision.FieldBall;
import vision.LaneSelector;
import vision.VisionConstants;

/**
 * Non-blocking vision scan + collection command.
 * Enables processors, captures frames across scheduler loops, picks lane, drives to it.
 * Ends when path completes or 3 balls detected.
 * Zero Thread.sleep().
 */
public class VisionCollectCommand extends CommandBase {
    private final ArtifactDetector detector;
    private final Follower follower;
    private final PathChain[] goToPaths;
    private final double maxPower;
    private ChosenPath chosenPath;

    private enum State { WARMUP, CAPTURING, DRIVING }
    private State state;
    private final ElapsedTime timer = new ElapsedTime();
    private final List<List<FieldBall>> capturedFrames = new ArrayList<>();
    private int framesCaptured;

    /** Readable from telemetry after the scan runs. */
    public static String lastScanResult = "No scan yet";

    public VisionCollectCommand(ArtifactDetector detector, Follower follower,
                                 PathChain[] goToPaths, double maxPower) {
        this.detector = detector;
        this.follower = follower;
        this.goToPaths = goToPaths;
        this.maxPower = maxPower;
    }

    @Override
    public void initialize() {
        capturedFrames.clear();
        framesCaptured = 0;
        state = State.WARMUP;
        detector.enable();
        timer.reset();
    }

    @Override
    public void execute() {
        switch (state) {
            case WARMUP:
                if (timer.milliseconds() >= VisionConstants.PROCESSOR_WARMUP_MS) {
                    captureFrame();
                    state = State.CAPTURING;
                    timer.reset();
                }
                break;

            case CAPTURING:
                if (timer.milliseconds() >= VisionConstants.INTER_FRAME_SLEEP_MS) {
                    captureFrame();
                    if (framesCaptured >= VisionConstants.NUM_SCAN_FRAMES) {
                        detector.disable();
                        decideLaneAndDrive();
                        state = State.DRIVING;
                    } else {
                        timer.reset();
                    }
                }
                break;

            case DRIVING:
                // Follower runs autonomously — nothing to do
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DRIVING && (!follower.isBusy() || isFull());
    }

    @Override
    public void end(boolean interrupted) {
        if (state != State.DRIVING) {
            // Aborted during scan — clean up
            detector.disable();
        }
        follower.breakFollowing();
    }

    public ChosenPath getChosenPath() {
        return chosenPath;
    }

    private void captureFrame() {
        List<Detection> raw = detector.scanOnce();
        if (raw == null) raw = new ArrayList<>();
        List<FieldBall> fieldBalls = new ArrayList<>();
        com.pedropathing.geometry.Pose pose = follower.getPose();
        for (Detection d : raw) {
            if (d != null) {
                fieldBalls.add(BallLocalizer.toFieldFrame(d, pose));
            }
        }
        capturedFrames.add(fieldBalls);
        framesCaptured++;
    }

    private void decideLaneAndDrive() {
        try {
            chosenPath = LaneSelector.selectFromFrames(capturedFrames, detector);
            lastScanResult = LaneSelector.lastScanDebug;
        } catch (Exception e) {
            chosenPath = VisionConstants.NO_DETECTION_FALLBACK;
            lastScanResult = "VISION ERROR: " + e.getClass().getSimpleName()
                    + " | fallback=" + chosenPath;
        }

        follower.setMaxPower(maxPower);
        follower.followPath(goToPaths[chosenPath.ordinal()], false);
    }

    private boolean isFull() {
        RobotHardware hw = RobotHardware.getInstance();
        return hw.spindexerSensorPair.quickCheck().ballPresent
                && hw.rampSensorPair.quickCheck().ballPresent
                && hw.transferSensorPair.quickCheck().ballPresent;
    }
}
