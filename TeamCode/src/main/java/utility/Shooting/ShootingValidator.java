package utility.Shooting;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import subsystems.Shooter;
import utility.RobotConstants.Enums.FieldState;

/**
 * ShootingValidator
 *
 * Validates whether the robot is allowed to shoot based on its position on the field.
 * Blocks shooting when outside the shooting zone, unless override is activated.
 *
 * Override Controls: DPAD_LEFT + RIGHT_BUMPER
 *
 * Usage:
 * - Call canShoot(overrideRequested) before allowing shooter to fire
 * - Returns true if shooting is allowed, false if blocked
 */
public class ShootingValidator {
    private final Shooter shooter;
    private final Telemetry telemetry;

    private long lastOverrideTime = 0;
    private static final long OVERRIDE_WARNING_THRESHOLD = 30000; // 30 seconds in milliseconds

    public ShootingValidator(Shooter shooter, Telemetry telemetry) {
        this.shooter = shooter;
        this.telemetry = telemetry;
    }


    public boolean canShoot(boolean overrideRequested) {
        // Get current field state from shooter subsystem
        // (Shooter.updateFieldState() runs in shooter.periodic(), so it's always up-to-date)
        FieldState currentFieldState = shooter.getFieldState();

        // Check if in shooting zone
        boolean inShootingZone = (currentFieldState == FieldState.ShootingZone);

        // Allow if in zone OR override is active
        if (inShootingZone) {
            telemetry.addData("Shooting", "ALLOWED (In Zone)");
            telemetry.addData("Field Zone", "Shooting Zone");
            return true;
        } else if (overrideRequested) {
            // Track override time for warning
            long currentTime = System.currentTimeMillis();
            if (lastOverrideTime == 0) {
                lastOverrideTime = currentTime;
            }

            telemetry.addData("Shooting", "OVERRIDE ACTIVE");
            telemetry.addData("Field Zone", currentFieldState.toString());

            // Warn if override held for too long
            if (currentTime - lastOverrideTime > OVERRIDE_WARNING_THRESHOLD) {
                telemetry.addData("WARNING", "Override held for >30 seconds!");
            }

            return true;
        } else {
            // Reset override timer when not in use
            lastOverrideTime = 0;

            telemetry.addData("Shooting", "BLOCKED (Outside Zone)");
            telemetry.addData("Field Zone", currentFieldState.toString());
            telemetry.addData("Override Hint", "Click RIGHT STICK to override");
            return false;
        }
    }


    public String getStatus(boolean overrideRequested) {
        FieldState currentFieldState = shooter.getFieldState();
        boolean inShootingZone = (currentFieldState == FieldState.ShootingZone);

        if (inShootingZone) {
            return "ALLOWED";
        } else if (overrideRequested) {
            return "OVERRIDE";
        } else {
            return "BLOCKED";
        }
    }


    public boolean isInShootingZone() {
        return shooter.getFieldState() == FieldState.ShootingZone;
    }
}
