package subsystems;

import Constants.RobotConstants;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import Constants.EnumConstants;
import Constants.LimelightConstants;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import utility.RobotHardware;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import Constants.OdometryConstants;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import static java.lang.Math.toRadians;

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

    private Pose2D redPose = new Pose2D(DistanceUnit.INCH,8, 8.5, AngleUnit.DEGREES, 0);
    private Pose2D bluePose = new Pose2D(DistanceUnit.INCH,135, 8.5, AngleUnit.DEGREES, 180);

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
        if (latestResult == null || !latestResult.isValid()) {
            return -1;
        }

        List<LLResultTypes.FiducialResult> fiducialResults = latestResult.getFiducialResults();

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

    /**
     * Toggles between Goal Tracking and Tag Tracking modes.
     * Tag Tracking mode enables AprilTag scanning for motif pattern detection.
     */
    public void toggleMode() {
        if (currentMode == EnumConstants.LimelightMode.GoalTracking) {
            setMode(EnumConstants.LimelightMode.TagTracking);
        } else {
            setMode(EnumConstants.LimelightMode.GoalTracking);
        }
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

    public int getDetectedTagId() {
        return detectedTagId;
    }

    // ==================== RELOCALIZATION ====================

    /** Cached limelight pose, updated every loop */
    private Pose limelightPose = null;
    public String lastRelocDebug = "no attempt yet";

    public void updateLimelightPose() {
        if (robot.limelight == null) return;

        // Convert heading from Pedro to InvertedFTC frame (same frame the output uses)
        // Using the Pedro library ensures the round-trip is consistent
        Pose pedroHeading = new Pose(0, 0, robot.cachedHeading, PedroCoordinates.INSTANCE);
        double ftcYaw = Math.toDegrees(
                pedroHeading.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE).getHeading());
        robot.limelight.updateRobotOrientation(ftcYaw);

        LLResult result = robot.limelight.getLatestResult();
        if (result != null && result.isValid()
                && !result.getFiducialResults().isEmpty()) {
            Pose3D botpose = result.getBotpose_MT2();
            if (botpose != null) {
                double xInches = (botpose.getPosition().x*(72*(1/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES;
                double yInches = (botpose.getPosition().y*(72*(1/LimelightConstants.METERS_TO_INCHES))) * LimelightConstants.METERS_TO_INCHES;
                double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS) + (Math.PI/2);

                limelightPose = new Pose(xInches, yInches, heading);

                lastRelocDebug = String.format("raw=(%.3fm, %.3fm, %.1f°) -> pedro=(%.1f, %.1f, %.1f°)",
                        botpose.getPosition().x, botpose.getPosition().y,
                        botpose.getOrientation().getYaw(AngleUnit.DEGREES),
                        limelightPose.getX(), limelightPose.getY(),
                        Math.toDegrees(limelightPose.getHeading()));
            }
        }
    }

    /**
     * @return true if relocalization succeeded, false if no cached pose available
     */
    public boolean relocalizePinpoint() {
        if(RobotConstants.Robot.allianceColor == EnumConstants.AllianceColor.Blue){
            robot.pinpoint.setPosition(bluePose);
        }else{
            robot.pinpoint.setPosition(redPose);
        }

        robot.pinpoint.update();
        limelightPose = null;
        return true;
    }

    public boolean relocalizePinpointApriltag() {
        if (limelightPose == null) {
            lastRelocDebug = "no limelight pose cached";
            return false;
        }
        robot.pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, limelightPose.getX(), limelightPose.getY(), AngleUnit.RADIANS, limelightPose.getHeading()));
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
            updateLimelightData();
            scanForMotifTag();
        } else {
            updateLimelightPose();
        }
    }
}
