package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Limelight;
import utility.RobotHardware;

/**
 * Instant relocalization command — applies the continuously-cached limelight pose
 * to the Pinpoint in a single loop iteration. No pipeline switching needed since
 * Pipeline 2 (localization) is now the default.
 */
public class RelocalizePinpointCommand extends CommandBase {
    private final Limelight limelight;

    public RelocalizePinpointCommand(Limelight limelight) {
        this.limelight = limelight;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        if (limelight.fetchPoseForRelocalization()) {
            RobotHardware.getInstance().relocalizationPending = true;
            limelight.relocalizePinpointApriltag();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
