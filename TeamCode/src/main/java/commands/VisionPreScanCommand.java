package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

import vision.ArtifactDetector;
import vision.BallLocalizer;
import vision.Detection;
import vision.FieldBall;
import vision.LaneSelector;
import vision.VisionConstants;

/**
 * Non-blocking command that captures vision frames across multiple scheduler loops.
 * Enables processors, waits for warmup, captures NUM_SCAN_FRAMES frames with
 * INTER_FRAME_SLEEP_MS spacing, then disables processors and stores results.
 * Zero Thread.sleep() — all timing via ElapsedTime checks in execute().
 */
public class VisionPreScanCommand extends CommandBase {
    private final ArtifactDetector detector;
    private final Follower follower;
    private final ElapsedTime timer = new ElapsedTime();
    private final List<List<FieldBall>> capturedFrames = new ArrayList<>();

    private enum State { WARMUP, CAPTURING, DONE }
    private State state;
    private int framesCaptured;

    public VisionPreScanCommand(ArtifactDetector detector, Follower follower) {
        this.detector = detector;
        this.follower = follower;
        // No subsystem requirements — doesn't conflict with shooter/spindexer/turret
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
                        state = State.DONE;
                    } else {
                        timer.reset();
                    }
                }
                break;

            case DONE:
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        detector.disable();
        if (!interrupted) {
            LaneSelector.storeFrames(capturedFrames);
        }
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
}
