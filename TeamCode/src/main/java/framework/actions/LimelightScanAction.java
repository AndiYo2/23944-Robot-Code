package framework.actions;

import framework.Action;
import subsystems.Limelight;
import Constants.EnumConstants;
import utility.RobotHardware;

/**
 * Action that toggles the limelight to scanning mode and waits for motif detection.
 * Completes when the limelight has detected a motif.
 */
public class LimelightScanAction implements Action {
    private final Limelight limelight;

    public LimelightScanAction(Limelight limelight) {
        this.limelight = limelight;
    }

    @Override
    public void start() {
        if(!RobotHardware.getInstance().limelight.isRunning()){
            RobotHardware.getInstance().limelight.start();
        }
        limelight.setMode(EnumConstants.LimelightMode.TagTracking);
    }

    @Override
    public void update() {
        // Limelight.periodic() is called in the main loop
    }

    @Override
    public boolean isComplete() {
        return limelight.isMotifDetected();
    }

    @Override
    public void end() {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return "LimelightScan";
    }
}
