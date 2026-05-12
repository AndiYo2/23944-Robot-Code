package tests;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.json.JSONObject;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;

import utility.RobotHardware;
import vision.ArtifactDetector;
import vision.CorridorPlanner;
import vision.CorridorSelector;
import vision.FieldBall;
import vision.VisionConstants;

/**
 * Vision tuning TeleOp.
 *
 * Live telemetry shows camera controls, frame timing, per-color raw blob
 * counts, filtered detections, and the corridor planner result.
 *
 * Gamepad:
 *   LB / RB             EXPOSURE_MS  ± 1
 *   LT / RT             WB_KELVIN    ± 100 (trigger &gt; 0.5 required, edge-detected)
 *   DPAD_LEFT / RIGHT   GAIN         ± 10
 *   X                   apply current (EXPOSURE_MS, GAIN, WB_KELVIN) to the camera
 *   A                   save values to /sdcard/FIRST/vision_tuning.json
 *   B                   reload from /sdcard/FIRST/vision_tuning.json and apply
 *   START               trigger one corridor scan (cycles 3 frames, runs planner)
 */
@TeleOp(name = "Vision Tuning", group = "Tests")
public class VisionTuningTeleOp extends OpMode {

    private static final String TUNING_FILE =
            Environment.getExternalStorageDirectory() + "/FIRST/vision_tuning.json";

    private RobotHardware robot;
    private ArtifactDetector detector;

    private boolean prevX, prevA, prevB, prevStart;
    private boolean prevLB, prevRB, prevLeft, prevRight;
    private boolean prevLT, prevRT;

    private String scanReport = "(press START to scan)";
    private CorridorPlanner.Sweep lastSweep = CorridorPlanner.Sweep.EMPTY;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private double avgLoopMs = 0.0;

    @Override
    public void init() {
        robot = RobotHardware.getInstance();
        robot.init(hardwareMap);
        detector = new ArtifactDetector();

        loadFromDisk();
        telemetry.addLine("Vision Tuning ready. Press START to run a scan.");
    }

    @Override
    public void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        avgLoopMs = 0.9 * avgLoopMs + 0.1 * loopMs;

        if (risingEdge(gamepad1.right_bumper, prevRB)) VisionConstants.EXPOSURE_MS =
                Math.min(204, VisionConstants.EXPOSURE_MS + 1);
        if (risingEdge(gamepad1.left_bumper,  prevLB)) VisionConstants.EXPOSURE_MS =
                Math.max(0, VisionConstants.EXPOSURE_MS - 1);
        prevRB = gamepad1.right_bumper;
        prevLB = gamepad1.left_bumper;

        boolean rt = gamepad1.right_trigger > 0.5f;
        boolean lt = gamepad1.left_trigger  > 0.5f;
        if (risingEdge(rt, prevRT)) VisionConstants.WB_KELVIN =
                Math.min(6500, VisionConstants.WB_KELVIN + 100);
        if (risingEdge(lt, prevLT)) VisionConstants.WB_KELVIN =
                Math.max(2000, VisionConstants.WB_KELVIN - 100);
        prevRT = rt;
        prevLT = lt;

        if (risingEdge(gamepad1.dpad_right, prevRight)) VisionConstants.GAIN =
                Math.min(255, VisionConstants.GAIN + 10);
        if (risingEdge(gamepad1.dpad_left,  prevLeft )) VisionConstants.GAIN =
                Math.max(0, VisionConstants.GAIN - 10);
        prevRight = gamepad1.dpad_right;
        prevLeft  = gamepad1.dpad_left;

        if (risingEdge(gamepad1.x, prevX)) {
            robot.applyCameraControls();
            scanReport = "Applied EXP=" + VisionConstants.EXPOSURE_MS
                    + "ms GAIN=" + VisionConstants.GAIN
                    + " WB=" + VisionConstants.WB_KELVIN + "K";
        }
        if (risingEdge(gamepad1.a, prevA)) {
            scanReport = saveToDisk();
        }
        if (risingEdge(gamepad1.b, prevB)) {
            scanReport = loadFromDisk();
        }
        if (risingEdge(gamepad1.start, prevStart)) {
            runScan();
        }
        prevX     = gamepad1.x;
        prevA     = gamepad1.a;
        prevB     = gamepad1.b;
        prevStart = gamepad1.start;

        telemetry.addData("LoopMs avg", "%.1f", avgLoopMs);
        telemetry.addLine();
        telemetry.addData("Exposure (LB/RB)", "%d ms", VisionConstants.EXPOSURE_MS);
        telemetry.addData("Gain     (DPAD L/R)", "%d", VisionConstants.GAIN);
        telemetry.addData("WB       (LT/RT)", "%d K", VisionConstants.WB_KELVIN);
        telemetry.addLine("X=apply  A=save  B=reload  START=scan");
        telemetry.addLine();
        telemetry.addData("Green blobs (raw)",  detector.lastGreenBlobCount);
        telemetry.addData("Purple blobs (raw)", detector.lastPurpleBlobCount);
        List<FieldBall> merged = CorridorSelector.lastMergedBalls;
        telemetry.addData("Merged balls", merged.size());
        for (int i = 0; i < merged.size(); i++) {
            FieldBall b = merged.get(i);
            boolean inCorridor = lastSweep.captured.contains(b);
            telemetry.addData("  [" + i + "] " + b.color + (inCorridor ? " ✓" : ""),
                    "x=%.1f y=%.1f conf=%.2f", b.fieldX, b.fieldY, b.confidence);
        }
        telemetry.addLine();
        telemetry.addData("Corridor", lastSweep.isEmpty() ? "EMPTY"
                : String.format("hdg=%.1f° cap=%d score=%.2f",
                        Math.toDegrees(lastSweep.headingRad),
                        lastSweep.captured.size(),
                        lastSweep.score));
        telemetry.addData("Report", scanReport);
    }

    private void runScan() {
        Pose fakePose = new Pose(0, 0, 0);
        lastSweep  = CorridorSelector.selectCorridor(detector, fakePose, 1.0e6, -1.0e6);
        scanReport = CorridorSelector.lastScanDebug;
    }

    private String saveToDisk() {
        try {
            File dir = new File(Environment.getExternalStorageDirectory(), "FIRST");
            if (!dir.exists() && !dir.mkdirs()) return "SAVE FAILED: mkdir";
            JSONObject obj = new JSONObject();
            obj.put("EXPOSURE_MS", VisionConstants.EXPOSURE_MS);
            obj.put("GAIN",        VisionConstants.GAIN);
            obj.put("WB_KELVIN",   VisionConstants.WB_KELVIN);
            try (FileOutputStream fos = new FileOutputStream(TUNING_FILE)) {
                fos.write(obj.toString(2).getBytes(StandardCharsets.UTF_8));
            }
            return "SAVED → " + TUNING_FILE;
        } catch (Exception e) {
            return "SAVE FAILED: " + e.getClass().getSimpleName() + " " + e.getMessage();
        }
    }

    private String loadFromDisk() {
        File f = new File(TUNING_FILE);
        if (!f.exists()) return "no saved values (file missing)";
        try (InputStream in = new FileInputStream(f)) {
            ByteArrayOutputStream buf = new ByteArrayOutputStream();
            byte[] chunk = new byte[1024];
            int n;
            while ((n = in.read(chunk)) > 0) buf.write(chunk, 0, n);
            String content = new String(buf.toByteArray(), StandardCharsets.UTF_8);
            JSONObject obj = new JSONObject(content);
            if (obj.has("EXPOSURE_MS")) VisionConstants.EXPOSURE_MS = obj.getLong("EXPOSURE_MS");
            if (obj.has("GAIN"))        VisionConstants.GAIN        = obj.getInt("GAIN");
            if (obj.has("WB_KELVIN"))   VisionConstants.WB_KELVIN   = obj.getInt("WB_KELVIN");
            robot.applyCameraControls();
            return "LOADED + applied";
        } catch (Exception e) {
            return "LOAD FAILED: " + e.getClass().getSimpleName() + " " + e.getMessage();
        }
    }

    private static boolean risingEdge(boolean now, boolean prev) { return now && !prev; }
}
