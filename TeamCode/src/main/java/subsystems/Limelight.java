package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import utility.RobotHardware;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
        if (latestResult == null) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();
        if (fiducialResults == null || fiducialResults.isEmpty()) {
            return -1;
        }

        for (int tagId = 21; tagId <= 23; tagId++) {
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                if (fr.getFiducialId() == tagId) {
                    EnumConstants.BallColor[] pattern =
                        LimelightConstants.getMotifPatternForTag(tagId);

                    if (pattern != null && pattern.length == 3) {
                        LimelightConstants.motifPattern.setBallPattern(
                            pattern[0], pattern[1], pattern[2]);
                        motifDetected = true;
                        detectedTagId = tagId;

                        LimelightConstants.manuallySlowedForScan = false;

                        setMode(EnumConstants.LimelightMode.GoalTracking);
                        return tagId;
                    }
                }
            }
        }

        return -1;
    }

    public void setMode(EnumConstants.LimelightMode mode) {
        currentMode = mode;
        if (mode == EnumConstants.LimelightMode.TagTracking) {
            switchToMotifPipeline();
        } else {
            switchToLocalizationPipeline();
        }
    }

    public EnumConstants.LimelightMode getCurrentMode() {
        return currentMode;
    }

    public void resetLimelight() {
        motifDetected = false;
        detectedTagId = -1;
        LimelightConstants.manuallySlowedForScan = true;

        LimelightConstants.motifPattern.setBallPattern(
            EnumConstants.BallColor.Purple,
            EnumConstants.BallColor.Green,
            EnumConstants.BallColor.Purple);

        setMode(EnumConstants.LimelightMode.GoalTracking);
    }

    public boolean isMotifDetected() {
        return motifDetected;
    }

    public void resetMotifDetection() {
        motifDetected = false;
        detectedTagId = -1;
    }

    public int getDetectedTagId() {
        return detectedTagId;
    }

    // ==================== RELOCALIZATION ====================

    /** Cached limelight pose, updated every loop */
    private Pose limelightPose = null;
    public String lastRelocDebug = "no attempt yet";

    public Pose getLimelightPose() {
        return limelightPose;
    }

    /**
     * Pushes the current robot heading to the Limelight so MT2 stays accurate.
     * Cheap — just sends one number. Called every loop in periodic().
     */
    public void pushHeadingToLimelight() {
        if (robot.limelight == null) return;
        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.cachedHeading) + 90);
    }

    /**
     * Fetches the latest Limelight MT2 pose on demand.
     * Call this only when relocalization is actually requested (not every loop).
     * Heading must already be pushed via periodic() for MT2 accuracy.
     *
     * @return true if a valid AprilTag-based pose was obtained
     */
    public boolean fetchPoseForRelocalization() {
        if (robot.limelight == null) return false;

        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double xp = (botpose.getPosition().y * LimelightConstants.METERS_TO_INCHES) + 72;
                double yp = 72 - (botpose.getPosition().x * LimelightConstants.METERS_TO_INCHES);
                limelightPose = new Pose(xp, yp, Math.toRadians(botpose.getOrientation().getYaw() - 90));

                lastRelocDebug = String.format("raw=(%.3fm, %.3fm, %.1f°) -> pedro=(%.1f, %.1f, %.1f°)",
                        botpose.getPosition().x, botpose.getPosition().y,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                        limelightPose.getX(), limelightPose.getY(),
                        Math.toDegrees(limelightPose.getHeading()));
                return true;
            }
        }

        lastRelocDebug = "no valid AprilTag pose";
        return false;
    }

    public boolean relocalizePinpointApriltag() {
        if (limelightPose == null) {
            lastRelocDebug = "no limelight pose cached";
            return false;
        }
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, limelightPose.getX(), limelightPose.getY(), AngleUnit.RADIANS, limelightPose.getHeading()));
        robot.pinpoint.update();
        lastRelocDebug = String.format("APPLIED (%.1f, %.1f)",
                limelightPose.getX(), limelightPose.getY());
        return true;
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
        if (currentMode == EnumConstants.LimelightMode.TagTracking && !motifDetected) {
            // Re-enforce motif pipeline each loop to prevent updateRobotOrientation()
            // from the previous GoalTracking cycle holding the Limelight on pipeline 2
            robot.limelight.pipelineSwitch(LimelightConstants.MOTIF_PIPELINE);
            updateLimelightData();
            scanForMotifTag();
        } else {
            // Just push heading so MT2 stays accurate for on-demand relocalization.
            // NOTE: ramp-scan pipeline (6) is driven manually by the auton — periodic()
            // does NOT touch pipelines here, so manual switches survive.
            pushHeadingToLimelight();
        }
    }
}
