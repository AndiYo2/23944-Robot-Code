package commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import Constants.DriveConstants;
import utility.RobotHardware;

/**
 * Toggles the park mechanism between extended and retracted.
 *
 * Extend sequence:  Beam extend → Kick extend → wait 1s → Beam retract
 * Retract sequence: Beam extend → Kick retract → wait 1s → Beam retract
 *
 * Each press toggles which sequence runs next.
 */
public class ParkCommand extends CommandBase {
    private final RobotHardware robot = RobotHardware.getInstance();
    private final ElapsedTime timer = new ElapsedTime();
    private boolean beamRetracted = false;

    private static boolean extended = false;

    @Override
    public void initialize() {
        beamRetracted = false;
        extended = !extended;

        robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_EXTENDED);
        timer.reset();
    }

    @Override
    public void execute() {
        if (!beamRetracted && timer.seconds() >= 1.0) {
            robot.kickServo.setPosition(extended ? DriveConstants.PARK_SERVO_EXTEND : DriveConstants.PARK_SERVO_RETRACT);
            robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_RETRACTED);
            beamRetracted = true;
        }
    }

    @Override
    public boolean isFinished() {
        return beamRetracted;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            robot.beamServo.setPosition(DriveConstants.BEAM_SERVO_RETRACTED);
            robot.kickServo.setPosition(DriveConstants.PARK_SERVO_RETRACT);
        }
    }
}
