package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import Constants.LimelightConstants;
import subsystems.Limelight;
import utility.RobotHardware;

/**
 * One-shot pipeline switch for the Limelight. Fires the switch and exits
 * immediately — does NOT wait for the pipeline to settle. Follow this with
 * a WaitCommand (or .delay() in the builder) to give the Limelight time to
 * produce valid frames on the new pipeline before the next scan.
 *
 * Typical latency for a Limelight 3A pipeline switch to produce valid output:
 *   - AprilTag pipelines: ~150-300 ms
 *   - Python/SnapScript pipelines: ~200-500 ms
 *
 * Use named convenience commands (rampScan / localization / motif) or pass an
 * explicit pipeline index.
 */
public class SetLimelightPipelineCommand extends InstantCommand {

    public static String lastPipelineDebug = "no switch yet";

    public SetLimelightPipelineCommand(Limelight limelight, int pipelineIndex) {
        super(() -> {
            if (RobotHardware.getInstance().limelight != null) {
                if (!RobotHardware.getInstance().limelight.isRunning()) {
                    RobotHardware.getInstance().limelight.start();
                }
                RobotHardware.getInstance().limelight.pipelineSwitch(pipelineIndex);
                lastPipelineDebug = "switched to pipeline " + pipelineIndex;
            } else {
                lastPipelineDebug = "limelight null, skipped switch to " + pipelineIndex;
            }
        });
    }

    public static SetLimelightPipelineCommand rampScan(Limelight limelight) {
        return new SetLimelightPipelineCommand(limelight, LimelightConstants.RAMP_SCAN_PIPELINE);
    }

    public static SetLimelightPipelineCommand localization(Limelight limelight) {
        return new SetLimelightPipelineCommand(limelight, LimelightConstants.LOCALIZATION_PIPELINE);
    }

    public static SetLimelightPipelineCommand motif(Limelight limelight) {
        return new SetLimelightPipelineCommand(limelight, LimelightConstants.MOTIF_PIPELINE);
    }
}