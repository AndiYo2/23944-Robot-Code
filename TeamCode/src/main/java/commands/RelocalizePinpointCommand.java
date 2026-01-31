package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import subsystems.Limelight;

/**
 * Command that switches to the localization pipeline, waits for valid data,
 * caches the limelight pose, applies it to the Pinpoint, then switches back.
 */
public class RelocalizePinpointCommand extends CommandBase {
    private final Limelight limelight;
    private final ElapsedTime timer;
    private boolean success;

    private static final double SETTLE_SECONDS = 0.15;
    private static final double TIMEOUT_SECONDS = 0.5;

    public RelocalizePinpointCommand(Limelight limelight) {
        this.limelight = limelight;
        this.timer = new ElapsedTime();
        this.success = false;
        addRequirements(limelight);
    }

    @Override
    public void initialize() {
        limelight.switchToLocalizationPipeline();
        timer.reset();
        success = false;
    }

    @Override
    public void execute() {
        if (timer.seconds() < SETTLE_SECONDS) {
            return;
        }
        // Cache fresh pose from localization pipeline, then apply
        limelight.updateLimelightPose();
        success = limelight.relocalizePinpoint();
    }

    @Override
    public boolean isFinished() {
        return success || timer.seconds() >= TIMEOUT_SECONDS;
    }

    @Override
    public void end(boolean interrupted) {
        limelight.switchToMotifPipeline();
    }

    public boolean wasSuccessful() {
        return success;
    }
}
