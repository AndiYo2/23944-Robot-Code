package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import Constants.EnumConstants.FieldState;
import Constants.FieldMap;
import Constants.RobotConstants;
import utility.RobotHardware;

public class Odometry extends SubsystemBase {
    // Hardware reference
    RobotHardware robot;

    // Field zone state - determines whether turret tracks goal or stays centered
    private FieldState fieldState = FieldState.IdleZone;

    // Throttle counter — only run updateFieldState() every N loops
    private int fieldStateCounter = 0;

    // Pre-allocated corner array for updateFieldState() to avoid GC pressure
    private final double[][] corners = new double[4][2];


    public Odometry() {
        this.robot = RobotHardware.getInstance();
    }

    public void updateFieldState() {
        double centerX = robot.cachedPoseX;
        double centerY = robot.cachedPoseY;
        double heading = robot.cachedHeading;
        double halfSize = RobotConstants.Robot.HALF_SIZE;

        // Robot corners in robot-relative coordinates [forward, right]
        corners[0][0] =  halfSize; corners[0][1] =  halfSize;  // Front-right
        corners[1][0] =  halfSize; corners[1][1] = -halfSize;  // Front-left
        corners[2][0] = -halfSize; corners[2][1] =  halfSize;  // Back-right
        corners[3][0] = -halfSize; corners[3][1] = -halfSize;  // Back-left

        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);

        boolean inShootingZone = false;
        for (double[] corner : corners) {
            double cornerX = centerX + (corner[0] * cosH + corner[1] * sinH);
            double cornerY = centerY + (corner[0] * sinH - corner[1] * cosH);

            if (FieldMap.getPosition(cornerX, cornerY) == 'S') {
                inShootingZone = true;
                break;
            }
        }

        fieldState = inShootingZone ? FieldState.ShootingZone : FieldState.IdleZone;
    }

    public FieldState getFieldState() {
        return fieldState;
    }

    @Override
    public void periodic() {
            fieldStateCounter = 0;
            updateFieldState();
    }
}
