package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import utility.RobotHardware;

import java.util.List;

/**
 * Limelight subsystem for AprilTag detection and goal tracking.
 *
 * STATIC STATE MUTATIONS:
 * This class modifies the following static fields in LimelightConstants:
 * - motifPattern: Updated when a motif AprilTag (21-23) is detected
 * - isLimelightDisabled: Set true after detection, false on reset/toggle
 * - manuallySlowedForScan: Controls shooter speed during scanning
 *
 * These mutations allow state to persist across OpModes for Auto->TeleOp transitions.
 * The scanForMotifTag() method is the primary source of these side effects.
 */
public class Limelight implements Subsystem {
    private RobotHardware robot;
    private LLResult latestResult;
    private EnumConstants.LimelightMode currentMode;
    private boolean motifDetected;
    private int detectedTagId;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        this.latestResult = null;
        this.currentMode = EnumConstants.LimelightMode.TagTracking;
        this.motifDetected = false;
        this.detectedTagId = -1;
    }

    /**
     * Updates cached Limelight data from the sensor
     */
    public void updateLimelightData() {
        if (robot.limelight != null && !LimelightConstants.isLimelightDisabled) {
            latestResult = robot.limelight.getLatestResult();
        }
    }

    /**
     * Scans for motif AprilTags (21, 22, 23) and updates pattern if found.
     *
     * SIDE EFFECTS (on detection):
     * - LimelightConstants.motifPattern: Updated with detected pattern
     * - LimelightConstants.isLimelightDisabled: Set to true
     * - LimelightConstants.manuallySlowedForScan: Set to false
     * - Limelight hardware: Stopped to conserve power
     * - currentMode: Switched to GoalTracking
     *
     * @return tag ID if detected (21-23), -1 if not detected
     */
    public int scanForMotifTag() {
        if (latestResult == null || !latestResult.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();

        for (int tagId = 21; tagId <= 23; tagId++) {
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == tagId) {
                    // Tag found! Update motif pattern
                    EnumConstants.BallColor[] pattern =
                        LimelightConstants.getMotifPatternForTag(tagId);

                    if (pattern != null && pattern.length == 3) {
                        LimelightConstants.motifPattern.setBallPattern(
                            pattern[0], pattern[1], pattern[2]);
                        motifDetected = true;
                        detectedTagId = tagId;

                        // Turn off Limelight after detection
                        robot.limelight.stop();
                        LimelightConstants.isLimelightDisabled = true;
                        LimelightConstants.manuallySlowedForScan = false;

                        // Auto-switch back to Goal Tracking mode
                        currentMode = EnumConstants.LimelightMode.GoalTracking;
                        return tagId;
                    }
                }
            }
        }

        return -1;
    }

    /**
     * Toggles between Goal Tracking and Tag Tracking modes
     */
    public void toggleMode() {
        if (currentMode == EnumConstants.LimelightMode.GoalTracking) {
            // Switch to Tag Tracking mode
            currentMode = EnumConstants.LimelightMode.TagTracking;

            // Re-enable Limelight if it was disabled
            if (LimelightConstants.isLimelightDisabled) {
                LimelightConstants.isLimelightDisabled = false;
                robot.limelight.start();
            }
        } else {
            // Switch back to Goal Tracking mode
            currentMode = EnumConstants.LimelightMode.GoalTracking;
        }
    }
    public void setMode(EnumConstants.LimelightMode mode) {
        currentMode = mode;
    }

    public EnumConstants.LimelightMode getCurrentMode() {
        return currentMode;
    }

    public void resetLimelight() {
        robot.limelight.start();
        LimelightConstants.isLimelightDisabled = false;
        currentMode = EnumConstants.LimelightMode.TagTracking;
        motifDetected = false;
        detectedTagId = -1;
        LimelightConstants.manuallySlowedForScan = true;

        // Reset motif pattern to default PGP to clear stale pattern from previous runs
        LimelightConstants.motifPattern.setBallPattern(
            EnumConstants.BallColor.Purple,
            EnumConstants.BallColor.Green,
            EnumConstants.BallColor.Purple);
    }

    public boolean isMotifDetected() {
        return motifDetected;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    @Override
    public void periodic() {
        // Update Limelight data if in Tag Tracking mode
        if (currentMode == EnumConstants.LimelightMode.TagTracking && !motifDetected) {
            updateLimelightData();
            scanForMotifTag();
        }
    }
}
