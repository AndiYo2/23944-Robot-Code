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

    private Pose limelightPose = null;
    public String lastRelocDebug = "no attempt yet";

    public Pose getLimelightPose() {
        return limelightPose;
    }

    public void pushHeadingToLimelight() {
        if (robot.limelight == null) return;
        robot.limelight.updateRobotOrientation(Math.toDegrees(robot.cachedHeading) + 90);
    }

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
            robot.limelight.pipelineSwitch(LimelightConstants.MOTIF_PIPELINE);
            updateLimelightData();
            scanForMotifTag();
        } else {
            pushHeadingToLimelight();
        }
    }
}
