package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import utility.RobotHardware;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/**
 * Limelight subsystem for AprilTag detection and field relocalization.
 * - Scans for motif AprilTags (21-23) to determine ball pattern
 * - Provides MegaTag2-based field pose for Pinpoint relocalization
 *
 * STATIC STATE MUTATIONS:
 * This class modifies the following static fields in LimelightConstants:
 * - motifPattern: Updated when a motif AprilTag (21-23) is detected
 * - manuallySlowedForScan: Controls shooter speed during scanning
 *
 * These mutations allow state to persist across OpModes for Auto->TeleOp transitions.
 */
public class Limelight extends SubsystemBase {
    private RobotHardware robot;
    private LLResult latestResult;
    private EnumConstants.LimelightMode currentMode;
    private boolean motifDetected;
    private int detectedTagId;

    public Limelight() {
        this.robot = RobotHardware.getInstance();
        this.latestResult = null;
        this.currentMode = EnumConstants.LimelightMode.GoalTracking;
        this.motifDetected = false;
        this.detectedTagId = -1;
    }

    /**
     * Updates cached Limelight data from the sensor
     */
    public void updateLimelightData() {
        if (robot.limelight != null) {
            latestResult = robot.limelight.getLatestResult();
        }
    }

    /**
     * Scans for motif AprilTags (21, 22, 23) and updates pattern if found.
     *
     * SIDE EFFECTS (on detection):
     * - LimelightConstants.motifPattern: Updated with detected pattern
     * - LimelightConstants.manuallySlowedForScan: Set to false
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
     * Toggles between Goal Tracking and Tag Tracking modes.
     * Tag Tracking mode enables AprilTag scanning for motif pattern detection.
     */
    public void toggleMode() {
        if (currentMode == EnumConstants.LimelightMode.GoalTracking) {
            // Switch to Tag Tracking mode
            currentMode = EnumConstants.LimelightMode.TagTracking;
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

    // ==================== RELOCALIZATION ====================

    /** Cached limelight pose, updated every loop */
    private Pose2D limelightPose = null;

    /** Debug string from the last relocalization attempt */
    public String lastRelocDebug = "no attempt yet";

    /**
     * Updates the cached limelight pose from getBotpose() every loop.
     * Matches Wmatistic's setLimelightPose() approach — continuously poll,
     * apply on demand via relocalizePinpoint().
     */
    public void updateLimelightPose() {
        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Limelight axes are rotated -90° from Pedro's
                // pedroX = LL_y, pedroY = -LL_x (then shift from field-center to bottom-left)
                double x = (botpose.getPosition().y * LimelightConstants.METERS_TO_INCHES)
                        + LimelightConstants.FIELD_CENTER_OFFSET_INCHES;
                double y = (-botpose.getPosition().x * LimelightConstants.METERS_TO_INCHES)
                        + LimelightConstants.FIELD_CENTER_OFFSET_INCHES;
                double headingRad = Math.toRadians(botpose.getOrientation().getYaw(AngleUnit.DEGREES) - 90);

                limelightPose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad);
                lastRelocDebug = String.format("raw=(%.3fm, %.3fm, %.1f°) -> pedro=(%.1f, %.1f, %.1f°)",
                        botpose.getPosition().x, botpose.getPosition().y,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                        x, y, Math.toDegrees(headingRad));
            }
        }
    }

    /**
     * Performs a hard-snap relocalization of the Pinpoint using the cached limelight pose.
     * @return true if relocalization succeeded, false if no cached pose available
     */
    public boolean relocalizePinpoint() {
        if (limelightPose == null) {
            lastRelocDebug = "no limelight pose cached";
            return false;
        }

        robot.pinpoint.setPosition(limelightPose);
        robot.pinpoint.update();
        lastRelocDebug = String.format("APPLIED (%.1f, %.1f)",
                limelightPose.getX(DistanceUnit.INCH), limelightPose.getY(DistanceUnit.INCH));
        return true;
    }

    public Pose2D getLimelightPose() {
        return limelightPose;
    }

    // ==================== PIPELINE SWITCHING ====================

    public void switchToLocalizationPipeline() {
        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(LimelightConstants.LOCALIZATION_PIPELINE);
        }
    }

    public void switchToMotifPipeline() {
        if (robot.limelight != null) {
            robot.limelight.pipelineSwitch(LimelightConstants.MOTIF_PIPELINE);
        }
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
