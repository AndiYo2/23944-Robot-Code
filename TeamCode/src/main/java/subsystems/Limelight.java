package subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import utility.RobotConstants;
import utility.RobotHardware;

import java.util.List;

public class Limelight implements Subsystem {
    private RobotHardware robot;
    private LLResult latestResult;
    private RobotConstants.Enums.LimelightMode currentMode;
    private boolean motifDetected;
    private int detectedTagId;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        this.latestResult = null;
        this.currentMode = RobotConstants.Enums.LimelightMode.TagTracking;
        this.motifDetected = false;
        this.detectedTagId = -1;
    }

    /**
     * Updates cached Limelight data from the sensor
     */
    public void updateLimelightData() {
        if (robot.limelight != null && !RobotConstants.Limelight.isLimelightDisabled) {
            latestResult = robot.limelight.getLatestResult();
        }
    }

    /**
     * Scans for motif AprilTags (21, 22, 23) and updates pattern if found
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
                    RobotConstants.Enums.BallColor[] pattern =
                        RobotConstants.Limelight.getMotifPatternForTag(tagId);

                    if (pattern != null && pattern.length == 3) {
                        RobotConstants.Limelight.motifPattern.setBallPattern(
                            pattern[0], pattern[1], pattern[2]);
                        motifDetected = true;
                        detectedTagId = tagId;

                        // Turn off Limelight after detection
                        robot.limelight.stop();
                        RobotConstants.Limelight.isLimelightDisabled = true;

                        // Auto-switch back to Goal Tracking mode
                        currentMode = RobotConstants.Enums.LimelightMode.GoalTracking;

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
        if (currentMode == RobotConstants.Enums.LimelightMode.GoalTracking) {
            // Switch to Tag Tracking mode
            currentMode = RobotConstants.Enums.LimelightMode.TagTracking;

            // Re-enable Limelight if it was disabled
            if (RobotConstants.Limelight.isLimelightDisabled) {
                RobotConstants.Limelight.isLimelightDisabled = false;
                robot.limelight.start();
            }
        } else {
            // Switch back to Goal Tracking mode
            currentMode = RobotConstants.Enums.LimelightMode.GoalTracking;
        }
    }

    public RobotConstants.Enums.LimelightMode getCurrentMode() {
        return currentMode;
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
        if (currentMode == RobotConstants.Enums.LimelightMode.TagTracking && !motifDetected) {
            updateLimelightData();
            scanForMotifTag();
        }
    }
}
