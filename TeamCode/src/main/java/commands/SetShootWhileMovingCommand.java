package commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import Constants.ShooterConstants;

/**
 * Command that enables or disables shoot-while-moving lead compensation.
 * Completes immediately - just toggles the static flag.
 */
public class SetShootWhileMovingCommand extends InstantCommand {

    public SetShootWhileMovingCommand(boolean enabled) {
        super(() -> ShooterConstants.SHOOT_WHILE_MOVING_ENABLED = enabled);
    }
}
