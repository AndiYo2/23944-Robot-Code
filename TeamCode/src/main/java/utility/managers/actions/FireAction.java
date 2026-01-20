package utility.managers.actions;

import com.qualcomm.robotcore.util.ElapsedTime;
import subsystems.Shooter;
import Constants.EnumConstants.FlickState;
import Constants.SpindexerConstants;

/**
 * Triggers the shooter.
 * - Waits for shooter to be ready before triggering (handles back-to-back shots)
 * - Waits for flipper time after triggering
 */
public class FireAction implements SpindexerAction {
    private final Shooter shooter;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean shotTriggered = false;

    public FireAction(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void start() {
        shotTriggered = false;
        timer.reset();
    }

    @Override
    public void update() {
        // Wait for shooter to be ready, then trigger
        if (!shotTriggered && shooter.getCurrentState() == FlickState.Idle) {
            shooter.triggerShot();
            shotTriggered = true;
            timer.reset();
        }
    }

    @Override
    public boolean isComplete() {
        if (!shotTriggered) {
            // Still waiting for shooter to be ready - timeout safety
            return timer.seconds() >= 0.5;
        }
        // Wait for shot to complete
        return timer.seconds() >= SpindexerConstants.SHOOTER_FLIPPER_OUT_TIME;
    }

    @Override
    public void end() {}

    @Override
    public String getName() {
        return shotTriggered ? "Fire" : "Fire[wait]";
    }
}
