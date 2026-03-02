package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.DriveConstants;
import utility.RobotHardware;

/**
 * Toggles the park mechanism between kick-out and kick-down.
 *
 * First press  (kick out):  Beam out → Kick out  → Beam in
 * Second press (kick down): Beam out → Kick down → Beam in
 *
 * Each press toggles which sequence runs next.
 */
public class ParkCommand extends CommandBase {
    private final RobotHardware robot = RobotHardware.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private int phase = 0; // 0=beam out, 1=kick, 2=beam in

    private static boolean extended = false;

    @Override
    public void initialize() {
        phase = 0;
        extended = !extended;

        // Phase 0: beam out
        robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_EXTENDED);
        timer.reset();
    }

    @Override
    public void execute() {
        if (phase == 0 && timer.seconds() >= 0.5) {
            // Phase 1: kick out or kick down
            robot.kickServo.setPosition(extended ? DriveConstants.PARK_SERVO_EXTEND : DriveConstants.PARK_SERVO_RETRACT);
            phase = 1;
            timer.reset();
        } else if (phase == 1 && timer.seconds() >= 0.5) {
            // Phase 2: beam in
            robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_RETRACTED);
            phase = 2;
        }
    }

    @Override
    public boolean isFinished() {
        return phase == 2;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_RETRACTED);
        }
    }
}
