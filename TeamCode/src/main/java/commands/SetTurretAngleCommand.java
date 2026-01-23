package commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import subsystems.Turret;

/**
 * Command to set the turret to a specific angle.
 * Useful for pre-aiming the turret before arriving at a shooting position.
 */
public class SetTurretAngleCommand extends InstantCommand {

    /**
     * Set the turret to a specific angle in degrees.
     *
     * @param turret the turret subsystem
     * @param degrees the target angle in turret degrees
     */
    public SetTurretAngleCommand(Turret turret, double degrees) {
        super(() -> turret.setTurretDegree(degrees), turret);
    }

    /**
     * Pre-aim the turret to the goal as if the robot were at the specified position.
     *
     * @param turret the turret subsystem
     * @param robotX hypothetical robot X position (inches)
     * @param robotY hypothetical robot Y position (inches)
     * @param robotHeadingDeg hypothetical robot heading (degrees)
     */
    public SetTurretAngleCommand(Turret turret, double robotX, double robotY, double robotHeadingDeg) {
        super(() -> {
            double angle = turret.getDegreesToGoalFromPosition(robotX, robotY, Math.toRadians(robotHeadingDeg));
            turret.setTurretDegree(angle);
        }, turret);
    }
}
